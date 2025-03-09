//External imports
#include <Arduino.h>
#include <math.h>
#include <Wire.h>

//Local imports
#include "comms/BusChain.h"
#include "comms/SerialInterface.h"
#include "actuators/ServoController.h"
#include "sensors/MagSensor.h"

// Cores to pin RTOS tasks to
uint8_t CORE_0 = 0;
uint8_t CORE_1 = 1;

// I2C pins
#define I2C_SDA_0 21
#define I2C_SCL_0 22
#define I2C_SDA_1 32
#define I2C_SCL_1 33

// BusChain address identifiers
uint8_t BUSCHAIN_0 = 0;
uint8_t BUSCHAIN_1 = 1;

// PWM output pin on channel 0 of servo driver used for servo synchronization
#define interruptPin 5

// Servo driver port index
#define servoDriverPort 3

// DRIFT motors to configure
#define NUM_MOTORS 3

//Serial baud rate
#define BAUD_RATE 921600

namespace SerialHeaders {
  	//Headers from master to controller

	//Pings microcontroller
	#define PING 0x1
	//Servo power update
	#define SERVO_POWER 0x2

	//Headers from controller to master

	//Acknowledges ping      
	#define PING_ACK 0x1
	//Sends sensor data
	#define SENSOR_DATA 0x2
	//PWM cycle start
	#define PWM_CYCLE 0xA                                                                                                
}

using namespace SerialHeaders;

// Servo channels for DRIFT motors
const uint8_t servoChannels[NUM_MOTORS] = {1, 2, 3};

// Servo power multiplier
const float servoPowerMultiplier = 1./32768.;

// Encoder ports on BusChain (servo, spool) per DRIFT motor
const uint8_t encoderPorts[NUM_MOTORS][2] = {{7, 6}, {8, 9}, {5, 4}};//{{0, 1}, {7, 6}, {8, 9}, {5, 4}};

// BusChain objects
BusChain busChains[2];

// Sensor objects
MagSensor magSensors[NUM_MOTORS*2];

// List of sensor numbers on each buschain
uint8_t sensorsByBus[2][8];

// Number of sensors on each bus
uint8_t sensorCountByBus[2] = {0, 0};

// Task and interrupt function prototypes
void IRAM_ATTR onPWMStart();
void TaskScheduler(void *pvParameters);
void TaskPWMCycle(void *pvParameters);
void TaskSensorRead(void *pvParameters);
void TaskServoController(void *pvParameters);
void TaskSerialInterface(void *pvParameters);

// Define task handles
TaskHandle_t schedulerHandles[2];
TaskHandle_t pwmCycleHandle;
TaskHandle_t sensorReadHandles[2];
TaskHandle_t servoControllerHandle;
TaskHandle_t serialInterfaceHandle;

// Define sensor data queue
typedef uint8_t sensorID_t;
QueueHandle_t sensorDataQueue;

uint64_t startTime;

volatile bool aliveFlag = false;
volatile bool pwmCycleFlag = false;

// The setup function runs once when you press reset or power on the board.
void setup() {
	// Initialize serial communication at 115200 bits per second:
	SerialInterface::begin(BAUD_RATE);

	//Configures built-in LED
	pinMode(LED_BUILTIN, OUTPUT);
	
	// Initialize I2C ports
	Wire.begin(I2C_SDA_0, I2C_SCL_0);
	Wire1.begin(I2C_SDA_1, I2C_SCL_1);

	// Sets bus parameters
	Wire.setTimeout(1000);
	Wire.setClock(1000000);
	Wire1.setTimeout(1000);
	Wire1.setClock(1000000);

	//Initializes buschain objects
	busChains[0].begin(&BUSCHAIN_0, &Wire);
	busChains[1].begin(&BUSCHAIN_1, &Wire1);

	// Initialize servo driver board, flags connection error
	if (!ServoController::begin(servoDriverPort % 8, &busChains[servoDriverPort >> 3])) {
		Serial.println("Error connecting to servo driver");
		while (true) {
			delay(1000);
		}
	}

	// Initialize encoders in order of id
	for (uint8_t i = 0; i < NUM_MOTORS * 2; i++) {
		// Finds encoder port for sensor id
		uint8_t encoderPort = encoderPorts[i / 2][i % 2];
		uint8_t bus = encoderPort >> 3;

		// Adds sensor id to list by bus
		sensorsByBus[bus][sensorCountByBus[bus]++] = i;

		// Attempts to connect through associated port and buschain
		if (!magSensors[i].begin(encoderPort % 8, &busChains[bus])) {
			Serial.print("Error connecting to encoder port: ");
			Serial.println(encoderPort);
			while (true) {
				delay(1000);
			}
		}
  	}

	sensorDataQueue = xQueueCreate(NUM_MOTORS*2, sizeof(sensorID_t));

	xTaskCreatePinnedToCore(TaskSerialInterface, "Serial Interface", 2048, NULL, 5, &serialInterfaceHandle, tskNO_AFFINITY);
	
	xTaskCreatePinnedToCore(TaskServoController, "Servo Controller", 2048, NULL, 4, &servoControllerHandle, servoDriverPort>>3);

	xTaskCreatePinnedToCore(TaskSensorRead, "Sensor Read 0", 2048, (void *)&CORE_0, 3, &sensorReadHandles[0], CORE_0);
	xTaskCreatePinnedToCore(TaskSensorRead, "Sensor Read 1", 2048, (void *)&CORE_1, 3, &sensorReadHandles[1], CORE_1);

	xTaskCreatePinnedToCore(TaskScheduler, "Scheduler 0", 2048, (void *)&CORE_0, 1, &schedulerHandles[0], CORE_0);
	xTaskCreatePinnedToCore(TaskScheduler, "Scheduler 1", 2048, (void *)&CORE_1, 1, &schedulerHandles[1], CORE_1);
	
	xTaskCreatePinnedToCore(TaskPWMCycle, "PWM Cycle", 2048, NULL, 2, &pwmCycleHandle, tskNO_AFFINITY);

	attachInterrupt(interruptPin, onPWMStart, RISING);
}

// Called on rising pwm interrupt pin
void IRAM_ATTR onPWMStart() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Updates pwm cycle timer for servo synchronization
	ServoController::updatePWMTime();

	// Notifies pwm cycle task to wake
	vTaskNotifyGiveFromISR(pwmCycleHandle, &xHigherPriorityTaskWoken);

	// Does context switching if notification wakes higher priority task than current
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/
void TaskScheduler(void *pvParameters) {
	uint8_t core = *((uint8_t*) pvParameters);
	for (;;) {
		if (Serial.available() > 0) {
			// If serial data needs to be received, yields to serial interface
			xTaskNotifyGive(serialInterfaceHandle);
		} else if (aliveFlag) {
			//Otherwise yields to sensor reading
			xTaskNotifyGive(sensorReadHandles[core]);
		}
		vTaskDelay(1);
	}
}

void TaskPWMCycle(void *pvParameters) {
	(void)pvParameters;
	for (;;) {
		// Waits for notification from interrupt
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		// Sets PWM cycle flag
		pwmCycleFlag = true;
		//Notifies serial interface
		xTaskNotifyGive(serialInterfaceHandle);

		// Updates PWM ranges based on servo powers
		for (uint8_t i = 0; i < NUM_MOTORS; i++) {
			ServoController::updatePWMCompute(servoChannels[i]);
		}
	}
}

void TaskSensorRead(void *pvParameters) {
	uint8_t bus = *((uint8_t*) pvParameters);
	for (;;) {
		for (uint8_t i = 0; i < sensorCountByBus[bus]; i++) {
			// Waits for notification from scheduler before every I2C transaction
			ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
			
			// Gets sensor id and updates from I2C
			sensorID_t sensorID = sensorsByBus[bus][i];
			magSensors[sensorID].update();

			// Adds sensor data to queue
			xQueueSend(sensorDataQueue, &sensorID, 0);
			// Notifies serial interface to send sensor data
			xTaskNotifyGive(serialInterfaceHandle);
		}
	}
}

void TaskServoController(void *pvParameters) {
	(void)pvParameters;
	for (;;) {
		// Waits for notification from serial interface that powers have been updated
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		// Sends PWM ranges to PWM driver over I2C
		for (uint8_t i = 0; i < NUM_MOTORS; i++) {
			ServoController::updatePWMDriver(servoChannels[i]);
		}
	}
}

void TaskSerialInterface(void *pvParameters) {
	(void)pvParameters;
	for (;;) {
		// Waits for notification from scheduler or sensor read
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		// Updates serial data
		SerialInterface::update();
		if (SerialInterface::headerReady()) {
			switch (SerialInterface::getHeader()) {
				case PING:
					aliveFlag = true;
					digitalWrite(LED_BUILTIN, HIGH);
					// Sends ping acknowledgement
					SerialInterface::sendByte(PING_ACK);
					SerialInterface::clearPacket();
					break;
				case SERVO_POWER:
					// Updates servo controller
					if (Serial.available() >= 3) {
						// Reads servo id and power
						uint8_t servoID = SerialInterface::readByte();
						int16_t val = SerialInterface::readData<int16_t>();
						float power = static_cast<float>(val)*servoPowerMultiplier;
						
						//Ensures servo ID and power are within ranges
						if ((servoID < sizeof(servoChannels)/sizeof(servoChannels[0])) && (abs(power) <= 1)) {
							// Sets servo power
							ServoController::setPower(servoChannels[servoID], power);
						}
						SerialInterface::clearPacket();
					} else if (SerialInterface::isPacketEnded()) {
						// Clears header if end of data frame reached
						SerialInterface::clearPacket();
						// Notifies servo controller to update PWM ranges
						xTaskNotifyGive(servoControllerHandle);
					}
					break;
				default:
					// Clears header if unknown header received
					SerialInterface::clearPacket();
					break;
			}
		} else if (aliveFlag && pwmCycleFlag) {
			// Notifies master on pwm cycle
			pwmCycleFlag = false;
			SerialInterface::sendByte(PWM_CYCLE);
		} else if (aliveFlag && uxQueueMessagesWaiting(sensorDataQueue) > 0) {
			// If no header to process, sends sensor data
		
			// Sends data header
			SerialInterface::sendByte(SENSOR_DATA);
			while (uxQueueMessagesWaiting(sensorDataQueue) > 0) {
				sensorID_t sensorID;
				xQueueReceive(sensorDataQueue, &sensorID, 0);
				// Sends sensor id
				SerialInterface::sendByte(sensorID);
				// Sends sensor data
				SerialInterface::sendInt16(magSensors[sensorID].rawY());
				SerialInterface::sendInt16(magSensors[sensorID].rawZ());
			}
			// Sends end of data frame
			SerialInterface::sendEnd();
		}
	}
}

void loop() {
  	// Empty loop
}
