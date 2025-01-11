/*
  ServoController.cpp - Uses PCA9685 PWM controller and meta-PWM for precise control of servos
  Created by Carson G. Ray
*/

#include "ServoController.h"

Adafruit_PWMServoDriver ServoController::pwmDriver = Adafruit_PWMServoDriver();
uint8_t ServoController::driverPort;
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

bool ServoController::begin(uint8_t driverPort, uint8_t interruptPin) {
	ServoController::driverPort = driverPort;

	reset();

	BusChain::selectPort(driverPort);
	bool ret = pwmDriver.begin();
	pwmDriver.setOscillatorFrequency(oscillatorFreq);
	pwmDriver.setPWMFreq(pwmFreq);
	pwmDriver.setPWM(0, 0, 100);
	BusChain::release();

	return ret;
}

void ServoController::reset() {
	taskENTER_CRITICAL(&spinlock);
	for (uint8_t i = 0; i < MAX_SERVOS; i++) {
		pwmStarts[i] = -1;
		basePWMs[i] = 0;
		criticalCounts[i] = 0;
	}
	taskEXIT_CRITICAL(&spinlock);
}

void ServoController::setPower(uint8_t channel, float power) {
	if (power > 1) {
		power = 1;
	} else if (power < -1) {
		power = -1;
	}
	int8_t dir = (int8_t) (abs(power)/power);
	int16_t scaled = round(power*rangeLength);
	int16_t intervals = scaled/deadband;
	uint16_t basePWM = rangeCenter + deadZone*dir + intervals*deadband;
	int8_t criticalCount = (int8_t) ((power*rangeLength - intervals*deadband)*deadbandRes/deadband);
	taskENTER_CRITICAL_ISR(&spinlock);
	basePWMs[channel] = basePWM;
	criticalCounts[channel] = criticalCount;
	taskEXIT_CRITICAL_ISR(&spinlock);
}

void ServoController::updatePWMTime() {
	taskENTER_CRITICAL_ISR(&spinlock);
	startTime = micros();
	taskEXIT_CRITICAL_ISR(&spinlock);
}

void ServoController::updatePWMCompute(uint8_t channel) {
	taskENTER_CRITICAL(&spinlock);
	pulseCount += 1;
	if (pulseCount == deadbandRes) {
		pulseCount = 0;
	}
	uint16_t basePWM = basePWMs[channel];
	int16_t criticalCount = criticalCounts[channel];
	taskEXIT_CRITICAL(&spinlock);
	if (basePWM > 0) {
		uint16_t pulseLength;
		if (pulseCount < abs(criticalCount)) {
			if (criticalCount > 0) {
				pulseLength = basePWM + deadband;
			} else {
				pulseLength = basePWM - deadband;
			}
		} else {
			pulseLength = basePWM;
		}
		if (pwmStarts[channel] < 0) {
			uint32_t timeDiff = map((micros() - startTime), 0, 20000, 0, 4096);
			pwmStarts[channel] = timeDiff + commsDelay;
		}
		pwmEnds[channel] = pwmStarts[channel] + pulseLength;
	}
}

void ServoController::updatePWMDriver(uint8_t channel) {
	if (pwmStarts[channel] > -1) {
		BusChain::selectPort(driverPort);
		pwmDriver.setPWM(channel + 1, pwmStarts[channel], pwmEnds[channel]);
		BusChain::release();
	}
}
