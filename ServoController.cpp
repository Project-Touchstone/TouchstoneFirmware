/*
  ServoController.cpp - Uses PCA9685 PWM controller and meta-PWM for precise control of servos
  Created by Carson G. Ray
*/

#include "ServoController.h"

Adafruit_PWMServoDriver ServoController::pwmDriver = Adafruit_PWMServoDriver();
uint8_t ServoController::driverPort;
uint32_t ServoController::pwmFreq = 50;
uint32_t ServoController::rangeCenter = 1500;
uint32_t ServoController::deadZone = 30;
uint32_t ServoController::rangeLength = 450;
uint16_t ServoController::deadband = 5;

uint32_t ServoController::pwmStart[MAX_SERVOS];
volatile bool ServoController::pulseFlag = false;
volatile uint16_t ServoController::pulseCount = 0;
volatile uint64_t ServoController::startTime;
uint32_t ServoController::commsDelay = 100;

portMUX_TYPE ServoController::interruptMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR ServoController::onPWMStart() {
  portENTER_CRITICAL_ISR(&interruptMux);
  startTime = micros();
  pulseFlag = true;
  pulseCount += 1;
  if (pulseCount == deadband - 1) {
    pulseCount = 0;
  }
  portEXIT_CRITICAL_ISR(&interruptMux);
}

uint32_t ServoController::microsToPWM(uint32_t micros) {
  uint32_t pwmPeriod = 1000000/pwmFreq;
  return map(micros, 0, pwmPeriod, 0, 4096);
}

void ServoController::begin(uint8_t driverPort, uint8_t interruptPin) {
    ServoController::driverPort = driverPort;

    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
      pwmStart[i] = 0;
    }

    attachInterrupt(interruptPin, &onPWMStart, RISING);

    BusChain::selectPort(driverPort);
    pwmDriver.begin();
    pwmDriver.setOscillatorFrequency(27000000);
    pwmDriver.setPWMFreq(pwmFreq);
    pwmDriver.setPWM(0, 0, 100);

    while (!checkPulseFlag()) {
      ;
    }
}

void ServoController::setPWM(uint8_t channel, uint32_t pulseLength) {
  uint32_t start;
  if (pwmStart[channel] > 0) {
    start = pwmStart[channel];
  } else {
    start = micros() - startTime + commsDelay;
    pwmStart[channel] = start;
  }
  uint32_t end = start + pulseLength;
  channel = channel + 1;
  BusChain::selectPort(driverPort);
  pwmDriver.setPWM(channel, microsToPWM(start), microsToPWM(end));
}

void ServoController::setPower(uint8_t channel, float power) {
    if (power > 1) {
        power = 1;
    } else if (power < -1) {
        power = -1;
    }
    int8_t dir = (int8_t) (abs(power)/power);
    int32_t micros = round(power*rangeLength);
    int32_t intervals = micros/deadband;
    uint32_t basePWM = rangeCenter + deadZone*dir + intervals*deadband;
    uint8_t criticalCount = (uint8_t) abs(micros - intervals*deadband);
    
    bool ret;
    if (pulseCount < criticalCount) {
      setPWM(channel, basePWM + deadband*dir);
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