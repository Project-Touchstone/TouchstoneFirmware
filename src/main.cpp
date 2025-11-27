//////////////////////////////////////////////////////////////
// Note: uncomment the following line to enable integration testing
// This will include an hpp file for testing purposes
// Be sure to comment out this line for production builds
//////////////////////////////////////////////////////////////
//#define INTEGRATION_TESTING

#ifdef INTEGRATION_TESTING
#include "../integration/foc_motor_test.hpp" // Testing file to run
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
#include "DynamicConfig.h"

//Internal library imports
#include "BusChain.h"
#include "RUDPCore.h"
#include "RUDPSerialNode.h"
#include "ServoController.h"
#include "HydraFOCMotor.h"
#include "MagSensor.h"
#include "MagEncoder.h"
#include "IMU.h"

using namespace InterfaceHeaders;
using Packet = std::shared_ptr<RUDPCore::Packet>;

//Dynamic configuration object
DynamicConfig config;

//Serial node object
RUDPSerialNode interface("Middleware Interface");
//Serial server protocol object
std::shared_ptr<RUDPCore> interfaceData;

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
void TaskComms(void *pvParameters);

// Interface read handler
void interfaceReadHandler(Packet packet);

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

// Whether configuration has occured
bool configFlag = false;

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

	// Gets data protocol
	interfaceData = interface.getProtocol();
	// Sets interface data handler
	interfaceData->setReadHandler(&interfaceReadHandler);

	// Begins connection
	interface.begin(SERIAL_BAUD_RATE);

	// RTOS task initialization
	servoQueue = xQueueCreate(config.numServos(), sizeof(servo_t));

	xTaskCreatePinnedToCore(TaskSensors, "Sensor Updates", 2048, NULL, 1, &sensorsHandle, CORE_1);
	xTaskCreatePinnedToCore(TaskServos, "Servo Updates", 2048, NULL, 2, &servosHandle, CORE_0);
	xTaskCreatePinnedToCore(TaskComms, "Communications", 2048, NULL, 1, &commsHandle, CORE_0);
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskSensors(void *pvParameters) {
	(void)pvParameters;
	// Busy loops until connection is live and configuration is finished
	while (!aliveFlag || !configFlag) {
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

void TaskComms(void *pvParameters) {
	(void)pvParameters;
	for (;;) {
		// Updates interface data
		interfaceData->updateData();
	}
}

void interfaceReadHandler(Packet packet) {
	//Reads serial packets
	switch (packet->getHeader()) {
		case PING: {
			// Sends acknowledgement
			auto pkt = interfaceData->createPacket(packet->getHeader());
			pkt.writeByte(ACK);
			interfaceData->sendPacket(pkt);

			aliveFlag = true;
			digitalWrite(LED_BUILTIN, HIGH);
			break;
		}
		case SENSOR_DATA: {
			// Sends affirmative response if configured
			auto pkt = interfaceData->createPacket(packet->getHeader());
			if (configFlag) {
				pkt.writeByte(ACK);
			} else {
				pkt.writeByte(NACK);
				interfaceData->sendPacket(pkt);
				break;
			}

			// Sends sensor data length
			pkt.writeByte(config.getSensorDataLength());

			// Sends magnetic encoder data
			for (uint8_t i = 0; i < config.numMagEncoders(); i++) {
				// Sends sensor data
				pkt.writeInt16(magEncoders[i].getRawAngle());
			}
			// Sends magnetic tracker data
			for (uint8_t i = 0; i < 2; i++) {
				//Sends tracker data
				pkt.writeInt16(magTrackers[i].rawX());
				pkt.writeInt16(magTrackers[i].rawY());
				pkt.writeInt16(magTrackers[i].rawZ());
			}
			// Sends imu data
			for (uint8_t i = 0; i < config.numIMUs(); i++) {
				//Sends imu data
				int16_t x, y, z;
				imus[i].getRawAccel(&x, &y, &z);
				pkt.writeInt16(x);
				pkt.writeInt16(y);
				pkt.writeInt16(z);
				imus[i].getRawGyro(&x, &y, &z);
				pkt.writeInt16(x);
				pkt.writeInt16(y);
				pkt.writeInt16(z);
			}
			// Sends packet
			interfaceData->sendPacket(pkt);
			break;
		}
		case SERVO_SIGNAL: {
			// Sends affirmative response if configured
			auto pkt = interfaceData->createPacket(packet->getHeader());
			if (configFlag) {
				pkt.writeByte(ACK);
				interfaceData->sendPacket(pkt);
			} else {
				pkt.writeByte(NACK);
				interfaceData->sendPacket(pkt);
				break;
			}

			// Reads servo id and signal
			uint8_t servoID = packet->readByte();
			int16_t val = packet->readData<int16_t>();
			float signal = static_cast<float>(val)*servoSignalDeserialize;

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
		}
		case FOC_VELOCITY: {
			// Sends affirmative response if configured
			auto pkt = interfaceData->createPacket(packet->getHeader());
			if (configFlag) {
				pkt.writeByte(ACK);
				interfaceData->sendPacket(pkt);
			} else {
				pkt.writeByte(NACK);
				interfaceData->sendPacket(pkt);
				break;
			}

			// Reads motor id and velocity data
			uint8_t motorId = packet->readByte();
			float vel = packet->readData<float>();

			// Sets velocity target
			focMotors[motorId].setVelocity(vel);
			break;
		}
		case FOC_TORQUE: {
			// Sends affirmative response if configured
			auto pkt = interfaceData->createPacket(packet->getHeader());
			if (configFlag) {
				pkt.writeByte(ACK);
				interfaceData->sendPacket(pkt);
			} else {
				pkt.writeByte(NACK);
				interfaceData->sendPacket(pkt);
				break;
			}
			
			// Reads motor id and torque data
			uint8_t motorId = packet->readByte();
			float torque = packet->readData<float>();

			// Sets position target
			focMotors[motorId].setTorque(torque);
			break;
		}
		case CONFIG_END: {
			// Sends acknowledgement
			auto pkt = interfaceData->createPacket(packet->getHeader());
			pkt.writeByte(ACK);
			interfaceData->sendPacket(pkt);
			// Configuration complete
			configFlag = true;
			break;
		}
		case CONFIG_BUSCHAIN: {
			// Reads I2C bus
			uint8_t i2cBus = packet->readByte();
			// Gets number of modules based on response length
			uint8_t numModules = packet->getPayloadLength();
			// Adds module ids to vector
			std::vector<uint8_t> moduleIds;
			for (uint8_t i = 0; i < numModules; i++) {
				moduleIds.push_back(packet->readByte());
			}
			// Adds configuration
			uint8_t id = config.addBusChain({i2cBus, moduleIds});
			// Begins buschain
			config.beginBusChain(id);
			// Sends acknowledgement
			auto pkt = interfaceData->createPacket(packet->getHeader());
			pkt.writeByte(ACK);
			interfaceData->sendPacket(pkt);
			break;
		}
		case CONFIG_MAG_ENCODER:
		case CONFIG_MAG_ENCODER_BC: {
			// Reads bus id
			uint8_t busId = packet->readByte();
			// Detemines whether it is on buschain or not
			bool onBusChain = false;
			uint8_t channel = 0;
			if (packet->getHeader() == CONFIG_MAG_ENCODER_BC) {
				onBusChain = true;
				channel = packet->readByte();
			}

			// Adds configuration
			DynamicConfig::I2CDeviceConfig i2cConfig = {onBusChain, busId, channel};
			config.addMagEncoder(i2cConfig);
			// Begins new mag encoder
			MagEncoder newMagEncoder = MagEncoder();
			magEncoders.push_back(newMagEncoder);

			auto pkt = interfaceData->createPacket(packet->getHeader());
			if (config.beginI2CDevice(i2cConfig, newMagEncoder)) {
				// Sends acknowledgement
				pkt.writeByte(ACK);
				interfaceData->sendPacket(pkt);
			} else {
				// Sends non acknowledge
				pkt.writeByte(NACK);
				interfaceData->sendPacket(pkt);
			}
			break;
		}
		case CONFIG_MAG_TRACKER:
		case CONFIG_MAG_TRACKER_BC: {
			// Reads bus id
			uint8_t busId = packet->readByte();
			// Detemines whether it is on buschain or not
			bool onBusChain = false;
			uint8_t channel = 0;
			if (packet->getHeader() == CONFIG_MAG_TRACKER_BC) {
				onBusChain = true;
				channel = packet->readByte();
			}

			// Adds configuration
			DynamicConfig::I2CDeviceConfig i2cConfig = {onBusChain, busId, channel};
			config.addMagTracker(i2cConfig);
			// Begins new mag encoder
			MagSensor newMagTracker = MagSensor();
			magTrackers.push_back(newMagTracker);

			auto pkt = interfaceData->createPacket(packet->getHeader());
			if (config.beginI2CDevice(i2cConfig, newMagTracker)) {
				// Sends acknowledgement
				pkt.writeByte(ACK);
				interfaceData->sendPacket(pkt);
			} else {
				// Sends non acknowledge
				pkt.writeByte(NACK);
				interfaceData->sendPacket(pkt);
			}
			break;
		}
		case CONFIG_IMU:
		case CONFIG_IMU_BC: {
			// Reads bus id
			uint8_t busId = packet->readByte();
			// Detemines whether it is on buschain or not
			bool onBusChain = false;
			uint8_t channel = 0;
			if (packet->getHeader() == CONFIG_MAG_ENCODER_BC) {
				onBusChain = true;
				channel = packet->readByte();
			}

			// Gets IMU-specific parameters
			uint8_t accelMode = packet->readByte();
			uint8_t gyroMode = packet->readByte();
			uint8_t filterMode = packet->readByte();

			// Adds configuration
			DynamicConfig::IMUConfig imuConfig = {onBusChain, busId, channel, accelMode, gyroMode, filterMode};
			uint8_t id = config.addIMU(imuConfig);
			// Begins new imu
			IMU newIMU = IMU();
			imus.push_back(newIMU);

			auto pkt = interfaceData->createPacket(packet->getHeader());
			if (config.beginIMU(id, newIMU)) {
				// Sends acknowledgement
				pkt.writeByte(ACK);
				interfaceData->sendPacket(pkt);
			} else {
				// Sends non acknowledge
				pkt.writeByte(NACK);
				interfaceData->sendPacket(pkt);
			}
			break;
		}
		case CONFIG_SERVO_DRIVER:
		case CONFIG_SERVO_DRIVER_BC: {
			// Reads bus id
			uint8_t busId = packet->readByte();
			// Detemines whether it is on buschain or not
			bool onBusChain = false;
			uint8_t channel = 0;
			if (packet->getHeader() == CONFIG_SERVO_DRIVER_BC) {
				onBusChain = true;
				channel = packet->readByte();
			}

			// Adds configuration
			DynamicConfig::I2CDeviceConfig i2cConfig = {onBusChain, busId, channel};
			config.addServoDriver(i2cConfig);
			// Begins new mag encoder
			ServoController newServoDriver = ServoController();
			servoDrivers.push_back(newServoDriver);

			auto pkt = interfaceData->createPacket(packet->getHeader());
			if (config.beginI2CDevice(i2cConfig, newServoDriver)) {
				// Sends acknowledgement
				pkt.writeByte(ACK);
				interfaceData->sendPacket(pkt);
			} else {
				// Sends non acknowledge
				pkt.writeByte(NACK);
				interfaceData->sendPacket(pkt);
			}
			break;
		}
		case CONFIG_SERVO: {
			// Reads servo driver id and channel
			uint8_t servoDriverId = packet->readByte();
			uint8_t channel = packet->readByte();

			// Adds configuration
			config.addServo({servoDriverId, channel});

			// Sends acknowledgement
			auto pkt = interfaceData->createPacket(packet->getHeader());
			pkt.writeByte(ACK);
			interfaceData->sendPacket(pkt);
			break;
		}
		case CONFIG_FOC_MOTOR: {
			// Reads FOC port
			config.addFOCMotor({packet->readByte()});

			// Sends acknowledgement
			auto pkt = interfaceData->createPacket(packet->getHeader());
			pkt.writeByte(ACK);
			interfaceData->sendPacket(pkt);
			break;
		}
	}
}

void loop() {
  	// Empty loop
}

#endif // INTEGRATION_TESTING
