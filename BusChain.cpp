/*
  BusChain.cpp - 64 I2C bus expansion using TCA9548A chips
  Created by Carson G. Ray
*/

#include "BusChain.h"

// Variable Declaration
uint8_t BusChain::ser;
uint8_t BusChain::clk;
uint8_t BusChain::rclk;
uint32_t BusChain::clockInterval = 1;
int8_t BusChain::lastGroup = -1;
SemaphoreHandle_t BusChain::mutex;

/// @brief Configures multiplexer chip addresses
/// @param ser Serial data pin for address configuration
/// @param clk Data clock pin for address configuration
/// @param rclk Latch address configuration pin
/// @param numGroups Number of 8 sensor multiplexer groups to enable
void BusChain::begin(uint8_t ser, uint8_t clk, uint8_t rclk, uint8_t numGroups) {
	//Initialize pins
	BusChain::ser = ser;
	BusChain::clk = clk;
	BusChain::rclk = rclk;
	pinMode(clk, OUTPUT);
	pinMode(rclk, OUTPUT);
	pinMode(ser, OUTPUT);

	//Initializes bus access mutex
	mutex = xSemaphoreCreateMutex();

	//Resets multiplexer chips
	sendAddressBits(0, true);
	uint8_t data = 0;
	for (uint8_t i = 0; i < numGroups; i++) {
		//Adds 3 bit group index as address and enable bit at the current group position
		data += (i << 1+(i%2)*4) + (1 << (i%2)*4);
		//Sends the current byte if two groups are populated or the last group was populated
		bool last = (i == numGroups-1);
		if (i%2 == 1 || last) {
			//If last is true, data is locked in
			sendAddressBits(data, last);
			data = 0;
		}
	}

	//Initializes I2C bus
	Wire.begin();
	Wire.setTimeout(1000);
	Wire.setClock(1000000);
}

/// @brief Opens I2C channel to port
/// @param port Port number (8 per sensor chip, 16 per BusChain board)
/// @return 0 (successful), >0 (error)
uint8_t BusChain::selectPort(uint8_t port) {
	xSemaphoreTake(mutex, portMAX_DELAY);
	//Gets current sensor group of 8 sensors
	uint8_t currGroup = port >> 3;
	//If the last sensor group was different, close the old channel
	if (lastGroup > -1 && currGroup != lastGroup) {
		Wire.beginTransmission(ROOT_ADDRESS + lastGroup);
		Wire.write(0);
		Wire.endTransmission();
	}
	//Updates last group
	lastGroup = currGroup;
	//Opens channel to current sensor group at the right port
	Wire.beginTransmission(ROOT_ADDRESS + currGroup);
	Wire.write(1 << port%8);
	return Wire.endTransmission();
}

/// @brief Allows other processes to access I2C bus
void BusChain::release() {
	xSemaphoreGive(mutex);
}

/// @brief Sends an address configuration byte
/// @param data byte data
/// @param lock whether shift register should be latched
void BusChain::sendAddressBits(uint8_t data, bool lock) {
	xSemaphoreTake(mutex, portMAX_DELAY);
	for (uint8_t i = 0; i < 8; i++) {
		//Sends one data bit
		digitalWrite(ser, data & 1);
		data = data >> 1;
		digitalWrite(clk, 1);
		//Latches if last bit and latch is enabled
		if (lock && i == 7) {
		digitalWrite(rclk, 1);
		}
		delayMicroseconds(clockInterval);
		//Pulls down signal
		digitalWrite(clk, 0);
		digitalWrite(rclk, 0);
		digitalWrite(ser, 0);
		delayMicroseconds(clockInterval);
	}
	xSemaphoreGive(mutex);
}