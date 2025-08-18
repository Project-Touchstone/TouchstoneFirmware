/*
  BusChain.cpp - 64 I2C bus expansion using TCA9548A chips
  Created by Carson G. Ray
*/

#include "BusChain.h"

/// @brief Initializes single BusChain configuration with default bus
/// @param busId address of multiplexing module
void BusChain::begin(uint8_t moduleId) {
  uint8_t moduleIds[1] = {moduleId};
  BusChain::begin(&moduleId, &Wire);
}

/// @brief Initializes BusChain configuration with specified bus
/// @param busIds addresses of multiplexing modules in chain
/// @param i2cPort specific port to use for I2C communication
void BusChain::begin(uint8_t* moduleIds, TwoWire* i2cBus) {
	//Initializes I2C bus
	i2cBus->begin();
	//Initializes bus access mutex
	mutex = xSemaphoreCreateMutex();
	//Initializes busIds and i2c port
	this->moduleIds = moduleIds;
	this->i2cBus = i2cBus;
}

/// @brief Opens I2C channel
/// @param channel Channel number (8 per board)
/// @return 0 (successful), >0 (error)
uint8_t BusChain::selectChannel(uint8_t channel) {
	xSemaphoreTake(mutex, portMAX_DELAY);
	//Gets identifier for current 8-sensor module in chain
	uint8_t currModule = moduleIds[channel >> 3];
	//If the last module was different, close the old channel
	if (lastModule > -1 && currModule != lastModule) {
		i2cBus->beginTransmission(ROOT_ADDRESS + lastModule);
		i2cBus->write(0);
		i2cBus->endTransmission();
	}
	//Updates last module opened
	lastModule = currModule;
	//Opens channel to current sensor group at the right port
	i2cBus->beginTransmission(ROOT_ADDRESS + currModule);
	i2cBus->write(1 << channel%8);
	return i2cBus->endTransmission();
}

/// @brief Allows other processes to access buschain
void BusChain::release() {
	xSemaphoreGive(mutex);
}

/// @brief Gets I2C bus
/// @return I2C bus
TwoWire* BusChain::getI2CBus() {
	return i2cBus;
}