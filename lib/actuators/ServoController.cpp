/*
  ServoController.cpp - Uses PCA9685 PWM controller and meta-PWM for precise control of servos
  Created by Carson G. Ray
*/

#include "ServoController.h"

Adafruit_PWMServoDriver ServoController::pwmDriver;;
BusChain* ServoController::busChain;
uint8_t ServoController::driverChannel;
const uint16_t ServoController::pwmFreq = 50;
const uint32_t ServoController::oscillatorFreq = 28300000;
const uint16_t ServoController::rangeCenter = 308;
const uint16_t ServoController::deadZone = 6;
const uint16_t ServoController::rangeLength = 102;
const uint16_t ServoController::deadband = 1;
const uint16_t ServoController::deadbandRes = 20;

int16_t ServoController::pwmStarts[MAX_SERVOS];
uint16_t ServoController::pwmEnds[MAX_SERVOS];
uint16_t ServoController::basePWMs[MAX_SERVOS];
int16_t ServoController::criticalCounts[MAX_SERVOS];
volatile uint8_t ServoController::pulseCount = 0;
volatile uint64_t ServoController::startTime;
uint16_t ServoController::commsDelay = 100;

portMUX_TYPE ServoController::spinlock = portMUX_INITIALIZER_UNLOCKED;

/// @brief Initializes ServoController object
/// @param driverChannel Channel number of servo driver on BusChain
/// @param busChain BusChain object
/// @return true (successful), false (error)
bool ServoController::begin(uint8_t driverChannel, BusChain* busChain) {
	// Initializes communication parameters
	ServoController::driverChannel = driverChannel;
	ServoController::busChain = busChain;

	// Initializes servo driver
	busChain->selectChannel(driverChannel);
	TwoWire* i2cPort = busChain->getI2CBus();
	pwmDriver = Adafruit_PWMServoDriver(DEFAULT_ADDRESS, *i2cPort);
	bool ret = pwmDriver.begin();
	pwmDriver.setOscillatorFrequency(oscillatorFreq);
	pwmDriver.setPWMFreq(pwmFreq);
	// Uses first channel to trigger interrupt
	pwmDriver.setPWM(0, 0, 100);
	busChain->release();

	// Resets pwm parameters
	reset();

	// Returns success
	return ret;
}

/// @brief Resets all servo parameters
void ServoController::reset() {
	taskENTER_CRITICAL(&spinlock);
	for (uint8_t i = 0; i < MAX_SERVOS; i++) {
		pwmStarts[i] = -1;
		basePWMs[i] = 0;
		criticalCounts[i] = 0;
	}
	taskEXIT_CRITICAL(&spinlock);
}

/// @brief Sets signal of servo
/// @param channel Servo channel number
/// @param power Signal value (-1 to 1)
void ServoController::setSignal(uint8_t channel, float power) {
	// Clamps power to range
	if (power > 1) {
		power = 1;
	} else if (power < -1) {
		power = -1;
	}
	// Determines direction of power
	int8_t dir = (int8_t) (abs(power)/power);
	// Scales power to range
	int16_t scaled = round(power*rangeLength);
	// Determines intervals of deadband width and base PWM
	int16_t intervals = scaled/deadband;
	uint16_t basePWM = rangeCenter + deadZone*dir + intervals*deadband;

	// Determines critical count for meta-PWM
	int8_t criticalCount = (int8_t) ((power*rangeLength - intervals*deadband)*deadbandRes/deadband);

	// Updates servo parameters
	taskENTER_CRITICAL_ISR(&spinlock);
	basePWMs[channel] = basePWM;
	criticalCounts[channel] = criticalCount;
	taskEXIT_CRITICAL_ISR(&spinlock);
}

/// @brief Updates start time of pwm cycle
void ServoController::updatePWMTime() {
	taskENTER_CRITICAL_ISR(&spinlock);
	startTime = micros();
	taskEXIT_CRITICAL_ISR(&spinlock);
}

/// @brief Updates PWM ranges for servo
/// @param channel Servo channel number
void ServoController::updatePWMCompute(uint8_t channel) {
	taskENTER_CRITICAL(&spinlock);
	pulseCount += 1;
	// Resets pulse count if it reaches deadband resolution
	if (pulseCount == deadbandRes) {
		pulseCount = 0;
	}
	// Retrieves servo parameters
	uint16_t basePWM = basePWMs[channel];
	int16_t criticalCount = criticalCounts[channel];
	taskEXIT_CRITICAL(&spinlock);

	if (basePWM > 0) {
		uint16_t pulseLength;
		// Adjusts pulse length based on whether pulsecount has passed critical count
		if (pulseCount < abs(criticalCount)) {
			if (criticalCount > 0) {
				pulseLength = basePWM + deadband;
			} else {
				pulseLength = basePWM - deadband;
			}
		} else {
			pulseLength = basePWM;
		}
		// Updates pwm start and end times
		if (pwmStarts[channel] < 0) {
			uint32_t timeDiff = map((micros() - startTime), 0, 20000, 0, 4096);
			pwmStarts[channel] = timeDiff + commsDelay;
		}
		pwmEnds[channel] = pwmStarts[channel] + pulseLength;
	}
}

/// @brief Updates PWM driver with new PWM ranges
void ServoController::updatePWMDriver(uint8_t channel) {
	if (pwmStarts[channel] > -1) {
		busChain->selectChannel(driverChannel);
		pwmDriver.setPWM(channel + 1, pwmStarts[channel], pwmEnds[channel]);
		busChain->release();
	}
}
