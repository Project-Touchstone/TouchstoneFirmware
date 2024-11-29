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

int16_t ServoController::pwmStart[MAX_SERVOS];
int16_t ServoController::basePWM[MAX_SERVOS];
int8_t ServoController::criticalCount[MAX_SERVOS];
volatile uint8_t ServoController::pulseCount = 0;
volatile bool ServoController::pulseFlag = false;
volatile uint64_t ServoController::startTime;
uint16_t ServoController::commsDelay = 100;

portMUX_TYPE ServoController::interruptMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR ServoController::onPWMStart() {
  portENTER_CRITICAL_ISR(&interruptMux);
  startTime = micros();
  pulseFlag = true;
  pulseCount += 1;
  if (pulseCount == deadbandRes) {
    pulseCount = 0;
  }
  portEXIT_CRITICAL_ISR(&interruptMux);
}

void ServoController::begin(uint8_t driverPort, uint8_t interruptPin) {
  ServoController::driverPort = driverPort;

  reset();

  attachInterrupt(interruptPin, &onPWMStart, RISING);

  BusChain::selectPort(driverPort);
  bool ret = pwmDriver.begin();
  pwmDriver.setOscillatorFrequency(oscillatorFreq);
  pwmDriver.setPWMFreq(pwmFreq);
  pwmDriver.setPWM(0, 0, 100);

  while (!checkPulseFlag()) {
    ;
  }

  return ret;
}

void ServoController::reset() {
  for (uint8_t i = 0; i < MAX_SERVOS; i++) {
    pwmStart[i] = -1;
    basePWM[i] = -1;
  }
}

void ServoController::setPWM(uint8_t channel, uint16_t pulseLength) {
  uint32_t start;
  if (pwmStart[channel] > -1) {
    start = pwmStart[channel];
  } else {
    start = micros() - startTime + commsDelay;
    pwmStart[channel] = start;
  }
  uint16_t end = start + pulseLength;
  channel = channel + 1;
  BusChain::selectPort(driverPort);
  if (pulseLength > 0) {
    pwmDriver.setPWM(channel, start, end);
  } else {
    pwmDriver.setPWM(4096, 0);
  }
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

  if (pulseCount < abs(criticalCount)) {
    if (criticalCount > 0) {
      setPWM(channel, basePWM + deadband);
    } else {
      setPWM(channel, basePWM - deadband);
    }
  } else {
    setPWM(channel, basePWM);
  }
}

bool ServoController::checkPulseFlag() {
  if (pulseFlag) {
    pulseFlag = false;
    return true;
  }
  return false;
}