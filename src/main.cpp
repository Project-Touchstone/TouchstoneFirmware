//////////////////////////////////////////////////////////////
// Note: uncomment the following line to enable integration testing
// This will include an hpp file for testing purposes
// Be sure to comment out this line for production builds
//////////////////////////////////////////////////////////////
//#define INTEGRATION_TESTING

#ifdef INTEGRATION_TESTING
#include "../integration/blink.hpp" // Testing file to run
#endif
//////////////////////////////////////////////////////////////

#ifndef INTEGRATION_TESTING
#define INTEGRATION_TESTING

//External imports
#include <Arduino.h>
#include <math.h>
#include <Wire.h>

//Configuration imports
#include "WindowConfig.h"
#include "SerialHeaders.h"

//Internal library imports
#include "BusChain.h"
#include "SerialInterface.h"
#include "ServoController.h"
#include "MagSensor.h"
#include "IMU.h"

using namespace SerialHeaders;

//TwoWire object
TwoWire I2C = TwoWire(0);

// BusChain object
BusChain busChain;

// Magnetic sensor objects
MagSensor magSensors[NUM_SERVOS*2];

//Magnetic tracker objects
MagSensor magTrackers[2];

// IMU object
IMU imu;

// Task and interrupt function prototypes
void IRAM_ATTR onPWMStart();
void TaskPWMCycle(void *pvParameters);
void TaskSensorCritical(void *pvParameters);
void TaskSensorNonCritical(void *pvParameters);
void TaskServoController(void *pvParameters);
void TaskSerialInterface(void *pvParameters);

// Define task handles
TaskHandle_t pwmCycleHandle;
TaskHandle_t sensorCriticalHandle;
TaskHandle_t sensorNonCriticalHandle;
TaskHandle_t servoControllerHandle;
TaskHandle_t serialInterfaceHandle;

// Define sensor data queue
typedef uint8_t sensorID_t;
QueueHandle_t sensorDataQueue;

// Define servo data queue
typedef uint8_t servoID_t;
QueueHandle_t servoDataQueue;

bool aliveFlag = false;
bool nonCriticalFlag = false;

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
	for (uint8_t i = 0; i < NUM_SERVOS * 2; i++) {
		// Finds encoder port for sensor id
		uint8_t encoderPort = encoderPorts[i / 2][i % 2];

		// Attempts to connect through associated port and buschain
		if (!magSensors[i].begin(encoderPort, &busChain)) {
			Serial.print("Error connecting to encoder port: ");
			Serial.println(encoderPort);
			while (true) {
				
			}
		}
  	}

	// Initializes IMU object
	imu.setParameters(MPU6050_RANGE_2_G, MPU6050_RANGE_250_DEG, MPU6050_BAND_260_HZ);
	if (!imu.begin(imuPort, &busChain)) {
		Serial.println("Error connecting to IMU");
		while (true) {
			
		}
	}

	// Initializes magnetic trackers
	for (uint8_t i = 0; i < 2; i++) {
		// Finds tracker port for sensor id
		uint8_t trackerPort = magTrackerPorts[i];

		// Attempts to connect through associated port and buschain
		if (!magTrackers[i].begin(trackerPort, &busChain)) {
			Serial.print("Error connecting to magnetic tracker port: ");
			Serial.println(trackerPort);
			while (true) {
				
			}
		}
  	}

	// Sets bus parameters
	I2C.setTimeout(I2C_TIMEOUT);
	I2C.setClock(I2C_BAUD_RATE);

	sensorDataQueue = xQueueCreate(NUM_SERVOS*2, sizeof(sensorID_t));
	servoDataQueue = xQueueCreate(NUM_SERVOS, sizeof(servoID_t));

	xTaskCreatePinnedToCore(TaskSerialInterface, "Serial Interface", 2048, NULL, 5, &serialInterfaceHandle, CORE_1);
	
	xTaskCreatePinnedToCore(TaskServoController, "Servo Controller", 2048, NULL, 4, &servoControllerHandle, CORE_0);

	xTaskCreatePinnedToCore(TaskSensorCritical, "Sensor Critical", 2048, NULL, 3, &sensorCriticalHandle, CORE_0);

	xTaskCreatePinnedToCore(TaskSensorNonCritical, "Sensor Non-critical", 2048, NULL, 4, &sensorNonCriticalHandle, CORE_0);
	
	xTaskCreatePinnedToCore(TaskPWMCycle, "PWM Cycle", 2048, NULL, 4, &pwmCycleHandle, CORE_1);

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
void TaskPWMCycle(void *pvParameters) {
	(void)pvParameters;
	for (;;) {
		// Waits for notification from interrupt
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		//Notifies serial interface
		xTaskNotifyGive(serialInterfaceHandle);
		//Notifies non-critical sensor task
		xTaskNotifyGive(sensorNonCriticalHandle);

		// Updates meta PWM based on previous servo powers
		for (uint8_t i = 0; i < NUM_SERVOS; i++) {
			ServoController::updatePWMCompute(servoChannels[i]);
		}
	}
}

void TaskSensorCritical(void *pvParameters) {
	(void)pvParameters;
	while (!aliveFlag) {
		vTaskDelay(1);
	}
	for (;;) {
		for (uint8_t i = 0; i < NUM_SERVOS*2; i++) {
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

void TaskSensorNonCritical(void *pvParameters) {
	(void)pvParameters;
	while (!aliveFlag) {
		vTaskDelay(1);
	}
	for (;;) {
		// Waits for notification from pwm cycle task
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		
		// Updates IMU
		imu.update();
		// Updates magnetic trackers
		for (uint8_t i = 0; i < 2; i++) {
			magTrackers[i].update();
		}

		// Sets flag to notify serial interface
		nonCriticalFlag = true;
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
					SerialInterface::writeByte(PING_ACK);
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
		} else if (aliveFlag && nonCriticalFlag) {
			// Sends non-critical sensor data
			// Sends IMU data

			// Data header
			SerialInterface::writeByte(IMU_DATA);
			//Sends imu id
			SerialInterface::writeByte(imuID);
			//Sends imu data
			int16_t x, y, z;
			imu.getRawAccel(&x, &y, &z);
			SerialInterface::writeInt16(x);
			SerialInterface::writeInt16(y);
			SerialInterface::writeInt16(z);
			imu.getRawGyro(&x, &y, &z);
			SerialInterface::writeInt16(x);
			SerialInterface::writeInt16(y);
			SerialInterface::writeInt16(z);

			// Sends magnetic tracker data
			for (uint8_t i = 0; i < 2; i++) {
				// Data header
				SerialInterface::writeByte(MAGTRACKER_DATA);
				//Sends tracker id
				SerialInterface::writeByte(i);
				//Sends tracker data
				SerialInterface::writeInt16(magTrackers[i].rawX());
				SerialInterface::writeInt16(magTrackers[i].rawY());
				SerialInterface::writeInt16(magTrackers[i].rawZ());
			}
			
			// Notifies master that we are ready for next pwm cycle
			nonCriticalFlag = false;
			SerialInterface::writeByte(PWM_CYCLE);
		} else if (aliveFlag && uxQueueMessagesWaiting(sensorDataQueue) > 0) {
			// If no header or non-critical sensor data to process, sends critical sensor data
		
			while (uxQueueMessagesWaiting(sensorDataQueue) > 0) {
				sensorID_t sensorID;
				xQueueReceive(sensorDataQueue, &sensorID, 0);
				// Sends data header
				SerialInterface::writeByte(MAGENCODER_DATA);
				// Sends sensor id
				SerialInterface::writeByte(sensorID);
				// Sends sensor data
				SerialInterface::writeInt16(magSensors[sensorID].rawY());
				SerialInterface::writeInt16(magSensors[sensorID].rawZ());
			}
		}
	}
}

void loop() {
  	// Empty loop
}

#endif // INTEGRATION_TESTING
