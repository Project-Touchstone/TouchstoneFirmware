/**
 * MagSensor.h - 3D magnetic sensor driver
 * Created by Carson G. Ray
 */

#include "MagSensor.h"

/// @brief Default constructor
MagSensor::MagSensor() {
	//Dynamic memory allocation for spinlock
	spinlock = (portMUX_TYPE*) malloc(sizeof(portMUX_TYPE));
	// Initialize the spinlock dynamically
	portMUX_INITIALIZE(spinlock);
	busChainEnable = false;
}

/// @brief Initializes MagSensor object using BusChain
/// @param sensorChannel Channel number of sensor on BusChain
/// @param busChain BusChain object
/// @return true (successful), false (error)
bool MagSensor::begin(uint8_t sensorChannel, BusChain* busChain) {
	this->sensorChannel = sensorChannel;
	this->busChain = busChain;

    // Ensures busChain is enabled
    busChainEnable = true;

	//Selects sensor channel and initializes sensor
	busChain->selectChannel(sensorChannel);
	bool ret = begin(busChain->getI2CBus());
    busChain->release();

	return ret;
}

/// @brief Initializes MagSensor object using direct I2C
/// @param wire I2C bus
/// @return true (successful), false (error)
bool MagSensor::begin(TwoWire* wire) {
	this->i2cBus = wire;
	magSensor.begin(*i2cBus);
	bool ret = magSensor.setAccessMode(magSensor.MASTERCONTROLLEDMODE);
	magSensor.disableTemp();
	return !ret;
}

/// @brief Updates sensor data
void MagSensor::update() {
    // If using BusChain, select the channel before reading
	if (busChainEnable) {
		busChain->selectChannel(sensorChannel);
	}
	magSensor.updateData();
    // Release the BusChain after reading
	if (busChainEnable) {
		busChain->release();
	}
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