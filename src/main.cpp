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
#include "InterfacePacketLengths.hpp"
#include "DynamicConfig.h"

//Internal library imports
#include "BusChain.h"
#include "MinBiTCore.h"
#include "MinBiTSerialNode.h"
#include "ServoController.h"
#include "HydraFOCMotor.h"
#include "MagSensor.h"
#include "MagEncoder.h"
#include "IMU.h"

using namespace InterfaceHeaders;
using Request = std::shared_ptr<MinBiTCore::Request>;

//Dynamic configuration object
DynamicConfig config;

//Serial server object
MinBiTSerialNode interface("Middleware Interface");
//Serial server protocol object
std::shared_ptr<MinBiTCore> interfaceData;

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

// Task function prototypes
void TaskSensors(void *pvParameters);
void TaskServos(void *pvParameters);
void TaskFOCMotors(void *pvParameters);
void TaskComms(void *pvParameters);

// Queues for sending actuator commands
typedef DynamicConfig::ServoConfig servo_t;
QueueHandle_t servoQueue;

// Define task handles
TaskHandle_t sensorsHandle;
TaskHandle_t servosHandle;
TaskHandle_t focMotorsHandle;
TaskHandle_t commsHandle;

// Whether serial connection is alive
bool aliveFlag = false;

// The setup function runs once when you press reset or power on the board.
void setup() {
	//Configures built-in LED
	pinMode(LED_BUILTIN, OUTPUT);
	
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

	// Sets interface data handler
	interface.setReadHandler(&interfaceReadHandler);
	// Gets data protocol
	interfaceData = interface.getProtocol();
	// Loads protocol info
	interfaceData->loadIncomingByRequest(&incomingByRequest);

	// Begins connection
	interface.begin(SERIAL_BAUD_RATE);

	// RTOS task initialization
	servoQueue = xQueueCreate(config.numServos(), sizeof(servo_t));

	xTaskCreatePinnedToCore(TaskSensors, "Sensor Updates", 2048, NULL, 3, &sensorsHandle, CORE_1);
	xTaskCreatePinnedToCore(TaskServos, "Servo Updates", 2048, NULL, 4, &servosHandle, CORE_0);
	xTaskCreatePinnedToCore(TaskComms, "Communications", 2048, NULL, 5, &commsHandle, CORE_0);
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskSensors(void *pvParameters) {
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

void TaskServos(void *pvParameters) {
	(void)pvParameters;
	for (;;) {
		// Waits for servos to be added to queue
		servo_t servoConfig;
		xQueueReceive(servoQueue, &servoConfig, portMAX_DELAY);

		// Sends PWM ranges to PWM driver over I2C
		uint8_t driverId = servoConfig.servoDriverId;
        uint8_t channel = servoConfig.channel;
		servoDrivers[driverId].updatePWMDriver(channel);
	}
}

void interfaceReadHandler(std::shared_ptr<MinBiTCore> protocol, Request request) {
	// Ensures request did not time out
	if (request->IsTimedOut())
	{
		return;
	}
	//Reads serial packets
	switch (request->GetHeader()) {
		case PING:
			aliveFlag = true;
			digitalWrite(LED_BUILTIN, HIGH);
			// Sends acknowledgement
			interfaceData->writeRequest(ACK);
			interfaceData->sendAll();
			break;
		case SENSOR_DATA:
			// Sends affirmative response
			interfaceData->writeRequest(ACK);

			// Sends sensor data length
			interfaceData->writeByte(config.getSensorDataLength());

			// Sends magnetic encoder data
			for (uint8_t i = 0; i < config.numMagEncoders(); i++) {
				// Sends sensor data
				interfaceData->writeInt16(magEncoders[i].getRawAngle());
			}
			// Sends magnetic tracker data
			for (uint8_t i = 0; i < 2; i++) {
				//Sends tracker data
				interfaceData->writeInt16(magTrackers[i].rawX());
				interfaceData->writeInt16(magTrackers[i].rawY());
				interfaceData->writeInt16(magTrackers[i].rawZ());
			}
			// Sends imu data
			for (uint8_t i = 0; i < config.numIMUs(); i++) {
				//Sends imu data
				int16_t x, y, z;
				imus[i].getRawAccel(&x, &y, &z);
				interfaceData->writeInt16(x);
				interfaceData->writeInt16(y);
				interfaceData->writeInt16(z);
				imus[i].getRawGyro(&x, &y, &z);
				interfaceData->writeInt16(x);
				interfaceData->writeInt16(y);
				interfaceData->writeInt16(z);
			}
			// Sends packet
			interfaceData->sendAll();
			break;
		case SERVO_SIGNAL:
			// Reads servo id and signal
			uint8_t servoID = interfaceData->readByte();
			int16_t val = interfaceData->readData<int16_t>();
			float signal = static_cast<float>(val)*servoSignalMultiplier;

			// Gets configuration
			DynamicConfig::ServoConfig servoConfig = config.getServo(servoID);
			
			// Ensures servo ID and signal are within ranges
			// Sets servo signal
			uint8_t driverId = servoConfig.servoDriverId;
        	uint8_t channel = servoConfig.channel;
			servoDrivers[driverId].setSignal(channel, signal);
			// Adds servo to update queue
			xQueueSend(servoQueue, &servoConfig, 0);
			break;
		case FOC_POSITION:
			// Reads motor id and position data
			uint8_t motorId = interfaceData->readByte();
			float pos = interfaceData->readData<float>();

			// Sets position target
			focMotors[motorId].setPosition(pos);
			break;
		case FOC_VELOCITY:
			// Reads motor id and velocity data
			uint8_t motorId = interfaceData->readByte();
			float vel = interfaceData->readData<float>();

			// Sets velocity target
			focMotors[motorId].setVelocity(vel);
			break;
		case FOC_TORQUE:
			// Reads motor id and torque data
			uint8_t motorId = interfaceData->readByte();
			float torque = interfaceData->readData<float>();

			// Sets position target
			focMotors[motorId].setTorque(torque);
			break;
		case CONFIG:
			// Turns on and off configuration mode somehow?
			break;
		case CONFIG_BUSCHAIN: {
			// Reads I2C bus
			uint8_t i2cBus = interfaceData->readByte();
			// Gets number of modules based on response length
			uint8_t numModules = request->GetResponseLength() - 1;
			// Adds module ids to vector
			std::vector<uint8_t> moduleIds;
			for (uint8_t i = 0; i < numModules; i++) {
				moduleIds.push_back(interfaceData->readByte());
			}
			// Adds configuration
			config.addBusChain({i2cBus, moduleIds});
			break;
		}
		case CONFIG_MAG_ENCODER:
		case CONFIG_MAG_ENCODER_BC:
			// Reads bus id
			uint8_t busId = interfaceData->readByte();
			// Detemines whether it is on buschain or not
			bool onBusChain = false;
			uint8_t channel = 0;
			if (request->GetHeader() == CONFIG_MAG_ENCODER_BC) {
				onBusChain = true;
				channel = interfaceData->readByte();
			}

			// Adds configuration
			config.addMagEncoder({onBusChain, busId, channel});
			break;
		case CONFIG_MAG_TRACKER:
		case CONFIG_MAG_TRACKER_BC:
			// Reads bus id
			uint8_t busId = interfaceData->readByte();
			// Detemines whether it is on buschain or not
			bool onBusChain = false;
			uint8_t channel = 0;
			if (request->GetHeader() == CONFIG_MAG_TRACKER_BC) {
				onBusChain = true;
				channel = interfaceData->readByte();
			}

			// Adds configuration
			config.addMagTracker({onBusChain, busId, channel});
			break;
		case CONFIG_IMU:
		case CONFIG_IMU_BC:
			// Reads bus id
			uint8_t busId = interfaceData->readByte();
			// Detemines whether it is on buschain or not
			bool onBusChain = false;
			uint8_t channel = 0;
			if (request->GetHeader() == CONFIG_MAG_ENCODER_BC) {
				onBusChain = true;
				channel = interfaceData->readByte();
			}

			// Gets IMU-specific parameters
			uint8_t accelMode = interfaceData->readByte();
			uint8_t gyroMode = interfaceData->readByte();
			uint8_t filterMode = interfaceData->readByte();

			// Adds configuration
			config.addIMU({onBusChain, busId, channel, accelMode, gyroMode, filterMode});
			break;
		case CONFIG_SERVO_DRIVER:
		case CONFIG_SERVO_DRIVER_BC:
			// Reads bus id
			uint8_t busId = interfaceData->readByte();
			// Detemines whether it is on buschain or not
			bool onBusChain = false;
			uint8_t channel = 0;
			if (request->GetHeader() == CONFIG_SERVO_DRIVER_BC) {
				onBusChain = true;
				channel = interfaceData->readByte();
			}

			// Adds configuration
			config.addServoDriver({onBusChain, busId, channel});
			break;
		case CONFIG_SERVO:
			// Reads servo driver id and channel
			uint8_t servoDriverId = interfaceData->readByte();
			uint8_t channel = interfaceData->readByte();

			// Adds configuration
			config.addServo({servoDriverId, channel});
			break;
		case CONFIG_FOC_MOTOR:
			// Reads FOC port
			config.addFOCMotor({interfaceData->readByte()});
			break;
	}
}

void loop() {
  	// Empty loop
}

#endif // INTEGRATION_TESTING
