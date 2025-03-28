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
#define I2C_SDA 21
#define I2C_SCL 22

// BusChain address identifiers
uint8_t busChainIDs[2] = {0, 1};

// PWM output pin on channel 0 of servo driver used for servo synchronization
#define interruptPin 5

// Servo driver port
#define servoDriverPort 3

// DRIFT motors to configure
#define NUM_MOTORS 4

//Serial baud rate
#define SERIAL_BAUD_RATE 115200//460800
#define I2C_BAUD_RATE 100000

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
const uint8_t servoChannels[NUM_MOTORS] = {0, 1, 2, 3};

// Servo power multiplier
const float servoPowerMultiplier = 1./32768.;

// Encoder ports on BusChain (servo, spool) per DRIFT motor
const uint8_t encoderPorts[NUM_MOTORS][2] = {{10, 11}, {0, 1}, {7, 6}, {8, 9}};

//TwoWire object
TwoWire I2C = TwoWire(0);

// BusChain object
BusChain busChain;

// Sensor objects
MagSensor magSensors[NUM_MOTORS*2];

// Task and interrupt function prototypes
void IRAM_ATTR onPWMStart();
void TaskPWMCycle(void *pvParameters);
void TaskSensorRead(void *pvParameters);
void TaskServoController(void *pvParameters);
void TaskSerialInterface(void *pvParameters);

// Define task handles
TaskHandle_t pwmCycleHandle;
TaskHandle_t sensorReadHandle;
TaskHandle_t servoControllerHandle;
TaskHandle_t serialInterfaceHandle;

// Define sensor data queue
typedef uint8_t sensorID_t;
QueueHandle_t sensorDataQueue;

// Define servo data queue
typedef uint8_t servoID_t;
QueueHandle_t servoDataQueue;

volatile bool aliveFlag = false;
volatile bool pwmCycleFlag = false;

// The setup function runs once when you press reset or power on the board.
void setup() {
	// Initialize serial communication at 115200 bits per second:
	SerialInterface::begin(SERIAL_BAUD_RATE);
	
	//Configures built-in LED
	pinMode(LED_BUILTIN, OUTPUT);
	
	// Initialize I2C port
	I2C.begin(I2C_SDA, I2C_SCL);

	//Initializes buschain object
	busChain.begin(busChainIDs, &I2C);

	// Initialize servo driver board, flags connection error
	if (!ServoController::begin(servoDriverPort, &busChain)) {
		Serial.println("Error connecting to servo driver");
		while (true) {
			
		}
	}

	// Initialize encoders in order of id
	for (uint8_t i = 0; i < NUM_MOTORS * 2; i++) {
		// Finds encoder port for sensor id
		uint8_t encoderPort = encoderPorts[i / 2][i % 2];
		uint8_t bus = encoderPort >> 3;

		// Attempts to connect through associated port and buschain
		if (!magSensors[i].begin(encoderPort, &busChain)) {
			Serial.print("Error connecting to encoder port: ");
			Serial.println(encoderPort);
			while (true) {
				
			}
		}
  	}

	// Sets bus parameters
	I2C.setTimeout(1000);
	I2C.setClock(I2C_BAUD_RATE);

	sensorDataQueue = xQueueCreate(NUM_MOTORS*2, sizeof(sensorID_t));
	servoDataQueue = xQueueCreate(NUM_MOTORS, sizeof(servoID_t));

	//xTaskCreatePinnedToCore(TaskSerialInterface, "Serial Interface", 2048, NULL, 5, &serialInterfaceHandle, CORE_1);
	
	//xTaskCreatePinnedToCore(TaskServoController, "Servo Controller", 2048, NULL, 4, &servoControllerHandle, CORE_0);

	//xTaskCreatePinnedToCore(TaskSensorRead, "Sensor Read", 2048, NULL, 3, &sensorReadHandle, CORE_0);
	
	//xTaskCreatePinnedToCore(TaskPWMCycle, "PWM Cycle", 2048, NULL, 4, &pwmCycleHandle, CORE_1);

	attachInterrupt(interruptPin, onPWMStart, RISING);
}

// Called on rising pwm interrupt pin
void IRAM_ATTR onPWMStart() {
	digitalWrite(LED_BUILTIN, HIGH);
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
	(void)pvParameters;
	while (!aliveFlag) {
		vTaskDelay(1);
	}
	for (;;) {
		for (uint8_t i = 0; i < NUM_MOTORS*2; i++) {
			// Gets sensor id and updates from I2C
			sensorID_t sensorID = i;
			magSensors[sensorID].update();
			
			// Adds sensor ID to queue
            xQueueSend(sensorDataQueue, &sensorID, 0);

			// Notifies serial interface to send sensor data
			xTaskNotifyGive(serialInterfaceHandle);
		}
	}
}

void TaskServoController(void *pvParameters) {
	(void)pvParameters;
	for (;;) {
		// Waits for servos to be added to queue
		servoID_t servoID;
		xQueueReceive(servoDataQueue, &servoID, portMAX_DELAY);

		// Sends PWM ranges to PWM driver over I2C
		ServoController::updatePWMDriver(servoChannels[servoID]);
	}
}

void TaskSerialInterface(void *pvParameters) {
	(void)pvParameters;
	for (;;) {
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
						// Adds servo to queue
						xQueueSend(servoDataQueue, &servoID, 0);
						// Clears header
						SerialInterface::clearPacket();
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
		
			while (uxQueueMessagesWaiting(sensorDataQueue) > 0) {
				sensorID_t sensorID;
				xQueueReceive(sensorDataQueue, &sensorID, 0);
				// Sends data header
				SerialInterface::sendByte(SENSOR_DATA);
				// Sends sensor id
				SerialInterface::sendByte(sensorID);
				// Sends sensor data
				SerialInterface::sendInt16(magSensors[sensorID].rawY());
				SerialInterface::sendInt16(magSensors[sensorID].rawZ());
			}
		}
	}
}

void loop() {
  	// Empty loop
}
