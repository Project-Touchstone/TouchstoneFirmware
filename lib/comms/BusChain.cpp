/*
  BusChain.cpp - 64 I2C bus expansion using TCA9548A chips
  Created by Carson G. Ray
*/

#include "BusChain.h"

/// @brief Initializes single BusChain configuration with default bus
/// @param busId address of multiplexing module
void BusChain::begin(uint8_t busId) {
  uint8_t busIds[1] = {busId};
  BusChain::begin(&busId, &Wire);
}

/// @brief Initializes BusChain configuration with specified bus
/// @param busIds addresses of multiplexing modules in chain
/// @param i2cPort specific port to use for I2C communication
void BusChain::begin(uint8_t* busIds, TwoWire* i2cPort) {
	//Initializes I2C bus
	i2cPort->begin();
	//Initializes bus access mutex
	mutex = xSemaphoreCreateMutex();
	//Initializes busIds and i2c port
	this->busIds = busIds;
	this->i2cPort = i2cPort;
}

/// @brief Opens I2C channel to port
/// @param port Port number (8 per board)
/// @return 0 (successful), >0 (error)
uint8_t BusChain::selectPort(uint8_t port) {
	xSemaphoreTake(mutex, portMAX_DELAY);
	//Gets identifier for current 8-sensor group in chain
	uint8_t currGroup = busIds[port >> 3];
	//If the last sensor group was different, close the old channel
	if (lastGroup > -1 && currGroup != lastGroup) {
		i2cPort->beginTransmission(ROOT_ADDRESS + lastGroup);
		i2cPort->write(0);
		i2cPort->endTransmission();
	}
	//Updates last group
	lastGroup = currGroup;
	//Opens channel to current sensor group at the right port
	i2cPort->beginTransmission(ROOT_ADDRESS + currGroup);
	i2cPort->write(1 << port%8);
	return i2cPort->endTransmission();
}

/// @brief Allows other processes to access I2C bus
void BusChain::release() {
	xSemaphoreGive(mutex);
}

/// @brief Gets I2C port
/// @return I2C port
TwoWire* BusChain::getI2CPort() {
	return i2cPort;
}