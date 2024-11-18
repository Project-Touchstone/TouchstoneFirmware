/*
  ModulatedServo.cpp - Increases speed resolution of continuous servo with modulated PWM signal
  Created by Carson G. Ray
*/

#include "ModulatedServo.h"

Servo ModulatedServo::servo = Servo();
uint32_t ModulatedServo::servoInterval = 20000;
uint32_t ModulatedServo::rangeCenter = 1500;
uint32_t ModulatedServo::deadZone = 30;
uint32_t ModulatedServo::rangeLength = 500;
uint16_t ModulatedServo::rangeInterval = 5;

uint32_t ModulatedServo::pwmLevel[2];
uint16_t ModulatedServo::criticalCount = 0;
volatile uint16_t ModulatedServo::pulseCount = 0;

hw_timer_t* ModulatedServo::timer = NULL;
portMUX_TYPE ModulatedServo::timerMux = portMUX_INITIALIZER_UNLOCKED;

void IRAM_ATTR ModulatedServo::onTimer() {
  portENTER_CRITICAL_ISR(&timerMux);
  if (pulseCount < criticalCount) {
    servo.writeMicroseconds(pwmLevel[1]);
  } else if (pulseCount == criticalCount) {
    servo.writeMicroseconds(pwmLevel[0]);
  }
  pulseCount+=1;
  if (pulseCount == rangeInterval - 1) {
    pulseCount = 0;
  }
  portEXIT_CRITICAL_ISR(&timerMux);
}

void ModulatedServo::attach(uint8_t pin) {
    servo.attach(pin);

    pwmLevel[0] = rangeCenter;

    timer = timerBegin(1000000);
    timerAttachInterrupt(timer, &onTimer);
    timerAlarm(timer, servoInterval, true, 0);
}

void ModulatedServo::drive(float power) {
    if (power > 1) {
        power = 1;
    } else if (power < -1) {
        power = -1;
    }
    int8_t dir = (int8_t) (abs(power)/power);
    int32_t micros = round(power*rangeLength);
    int32_t intervals = micros/rangeInterval;
    pwmLevel[0] = rangeCenter + deadZone*dir + intervals*rangeInterval;
    pwmLevel[1] = pwmLevel[0] + rangeInterval*dir;
    criticalCount = abs(micros - intervals*rangeInterval);
}