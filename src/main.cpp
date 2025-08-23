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
#include <vector>
#include <math.h>
#include <Wire.h>

//Configuration imports
#include "HydraFOCConfig.h"
#include "InterfaceHeaders.h"
#include "InterfaceRequestLengths.hpp"
#include "DynamicConfig.h"

//Internal library imports
#include "BusChain.h"
#include "MinBiTCore.h"
#include "MinBiTSerialServer.h"
#include "ServoController.h"
#include "HydraFOCMotor.h"
#include "MagSensor.h"
#include "MagEncoder.h"
#include "IMU.h"

using namespace InterfaceHeaders;

//Dynamic configuration object
DynamicConfig config;

//TwoWire objects
TwoWire I2CBuses[2] = {TwoWire(0), TwoWire(1)};

// BusChain objects for each bus
// note: there may not be a physical BusChain on each bus
BusChain busChains[2];

// Vectors of I2C device objects
std::vector<MagEncoder> magEncoders;
std::vector<MagSensor> magTrackers;
std::vector<IMU> imus;
std::vector<ServoController> servoDrivers;

// Vector of FOC motor objects
std::vector<HydraFOCMotor> focMotors;

// Task and interrupt function prototypes
void IRAM_ATTR onPWMStart();
void TaskPWMCycle(void *pvParameters);
void TaskSensorUpdate(void *pvParameters);
void TaskServoController(void *pvParameters);
void TaskSerialInterface(void *pvParameters);

// Queue for sending servo commands
typedef DynamicConfig::ServoConfig servoID_t;
QueueHandle_t servoDataQueue;

// Define task handles
TaskHandle_t pwmCycleHandle;
TaskHandle_t sensorCriticalHandle;
TaskHandle_t sensorNonCriticalHandle;
TaskHandle_t servoControllerHandle;
TaskHandle_t serialInterfaceHandle;

// Whether serial connection is alive
bool aliveFlag = false;

// The setup function runs once when you press reset or power on the board.
void setup() {
	//Configures built-in LED
	pinMode(LED_BUILTIN, OUTPUT);

	// Initialize serial communication at 115200 bits per second:
	SerialInterface::begin(SERIAL_BAUD_RATE);
	
	// Initialize I2C ports
	I2CBuses[0].begin(I2C0_SDA, I2C0_SCL);
	I2CBuses[1].begin(I2C1_SDA, I2C1_SCL);

	// Sets bus parameters
	for (uint8_t i = 0; i < 2; i++) {
		I2CBuses[i].setTimeout(I2C_TIMEOUT);
		I2CBuses[i].setClock(I2C_BAUD_RATE);
	}

	// Sets configuration I2C buses and buschains
	config.setI2CBuses(I2CBuses);
    config.setBusChains(busChains);

	// RTOS task initialization
	servoDataQueue = xQueueCreate(config.numServos(), sizeof(servoID_t));

	xTaskCreatePinnedToCore(TaskSerialInterface, "Serial Interface", 2048, NULL, 5, &serialInterfaceHandle, CORE_1);
	xTaskCreatePinnedToCore(TaskServoController, "Servo Controller", 2048, NULL, 4, &servoControllerHandle, CORE_0);
	xTaskCreatePinnedToCore(TaskSensorUpdate, "Sensor Critical", 2048, NULL, 3, &sensorCriticalHandle, CORE_0);
	xTaskCreatePinnedToCore(TaskPWMCycle, "PWM Cycle", 2048, NULL, 4, &pwmCycleHandle, CORE_1);
}

// Called on rising pwm interrupt pin
void IRAM_ATTR onPWMStart() {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	// Updates pwm cycle timer for servo synchronization
	for (uint8_t i = 0; i < config.numServoDrivers(); i++) {
		servoDrivers[i].updatePWMTime();
	}

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
		for (uint8_t i = 0; i < config.numServos(); i++) {
			DynamicConfig::ServoConfig servoConfig = config.getServo(i);
			uint8_t driverId = servoConfig.servoDriverId;
			uint8_t channel = servoConfig.channel;
			servoDrivers[driverId].updatePWMCompute(channel);
		}
	}
}

void TaskSensorUpdate(void *pvParameters) {
	(void)pvParameters;
	while (!aliveFlag) {
		vTaskDelay(1);
	}
	for (;;) {
		// Updates magnetic encoders
		for (uint8_t i = 0; i < config.numMagEncoders(); i++) {
			// Updates from I2C
			magEncoders[i].update();
		}
		// Updates magnetic trackers
		for (uint8_t i = 0; i < config.numMagTrackers(); i++) {
			// Updates from I2C
			magTrackers[i].update();
		}
		// Updates IMUs
		for (uint8_t i = 0; i < config.numMagTrackers(); i++) {
			// Updates from I2C
			imus[i].update();
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
		uint8_t driverId = servoID.servoDriverId;
        uint8_t channel = servoID.channel;
		servoDrivers[driverId].updatePWMDriver(channel);
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
				case SERVO_SIGNAL:
					// Updates servo controller
					if (Serial.available() >= 3) {
						// Reads servo id and signal
						uint8_t servoID = SerialInterface::readByte();
						int16_t val = SerialInterface::readData<int16_t>();
						float signal = static_cast<float>(val)*servoSignalMultiplier;
						
						//Ensures servo ID and signal are within ranges
						if ((servoID < sizeof(servoChannels)/sizeof(servoChannels[0])) && (abs(signal) <= 1)) {
							// Sets servo power
							servoController.setSignal(servoChannels[servoID], signal);
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
				SerialInterface::writeInt16(magEncoders[sensorID].rawY());
				SerialInterface::writeInt16(magEncoders[sensorID].rawZ());
			}
		}
	}
}

void loop() {
  	// Empty loop
}

#endif // INTEGRATION_TESTING
