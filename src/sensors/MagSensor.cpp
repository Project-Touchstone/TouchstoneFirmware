/**
 * MagSensor.h - Custom ring-based multi-magnet encoder implementation
 * Created by Carson G. Ray
 */

#include "MagSensor.h"

/// @brief Default constructor
MagSensor::MagSensor() {
	//Dynamic memory allocation for spinlock
	spinlock = (portMUX_TYPE*) malloc(sizeof(portMUX_TYPE));
	// Initialize the spinlock dynamically
	portMUX_INITIALIZE(spinlock);
}

/// @brief Initializes MagSensor object
/// @param encoderPort Port number of encoder on BusChain
/// @param busChain BusChain object
/// @return true (successful), false (error)
bool MagSensor::begin(uint8_t encoderPort, BusChain* busChain) {
	//Initializes communication parameters
	this->encoderPort = encoderPort;
	this->busChain = busChain;
	this->i2cPort = busChain->getI2CPort();

	//Selects encoder port and initializes sensor
	busChain->selectPort(encoderPort);
	magSensor.begin(*i2cPort);
	//Sets sensor to master-controlled mode
	bool ret = magSensor.setAccessMode(magSensor.MASTERCONTROLLEDMODE);
	// No temperature data is needed
	magSensor.disableTemp();

	busChain->release();

	//Returns success
	return !ret;
}

/// @brief Updates sensor data
void MagSensor::update() {
  busChain->selectPort(encoderPort);
  magSensor.updateData();
  busChain->release();
}

/// @brief Gets X-axis magnetic field
float MagSensor::getX() {
    taskENTER_CRITICAL(spinlock);
    float x = magSensor.getX();
    taskEXIT_CRITICAL(spinlock);
    return x;
}

/// @brief Gets Y-axis magnetic field
float MagSensor::getY() {
    taskENTER_CRITICAL(spinlock);
    float y = magSensor.getY();
    taskEXIT_CRITICAL(spinlock);
    return y;
}

/// @brief Gets Z-axis magnetic field
float MagSensor::getZ() {
    taskENTER_CRITICAL(spinlock);
    float z = magSensor.getZ();
    taskEXIT_CRITICAL(spinlock);
    return z;
}

/// @brief Gets raw X-axis magnetic field data
int16_t MagSensor::rawX() {
    taskENTER_CRITICAL(spinlock);
    int16_t rawX = magSensor.rawX();
    taskEXIT_CRITICAL(spinlock);
    return rawX;
}

/// @brief Gets raw Y-axis magnetic field data
int16_t MagSensor::rawY() {
    taskENTER_CRITICAL(spinlock);
    int16_t rawY = magSensor.rawY();
    taskEXIT_CRITICAL(spinlock);
    return rawY;
}

/// @brief Gets raw Z-axis magnetic field data
int16_t MagSensor::rawZ() {
    taskENTER_CRITICAL(spinlock);
    int16_t rawZ = magSensor.rawZ();
    taskEXIT_CRITICAL(spinlock);
    return rawZ;
}