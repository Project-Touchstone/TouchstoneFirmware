/*
  DRIFTMotor.cpp - Dynamic resistance integrated force-feedback and tracking motor
  Created by Carson G. Ray
*/

#include "DRIFTMotor.h"

DRIFTMotor::DRIFTMotor() {
  pid.SetOutputLimits(0, 1);
  pid.SetSampleTime(20);
}
uint16_t DRIFTMotor::attach(uint8_t servoPin, uint16_t encoderBus0, uint16_t encoderBus1) {
  ModulatedServo::attach(servoPin);

  for (uint8_t i = 0; i < 2; i++) {
    uint16_t bus;
    switch(i) {
      case 0:
        bus = encoderBus0;
        break;
      case 1:
        bus = encoderBus1;
        break;
    }
    encoderBuses[i] = bus;
    if (BusChain::selectBus(bus) != 0) {
      return bus;
    }
    
    if (encoders[i].begin()) {
      return bus;
    }
    encoders[i].setUnitsPerRadian(unitsPerRadian);
    encoders[i].setDirection(encoderDirs[i]);

    pid.SetMode(AUTOMATIC);
  }
}
bool DRIFTMotor::calibrate() {
  if (mode != CALIBRATION) {
    mode = CALIBRATION;
    timeStart = millis();
    ModulatedServo::drive(-0.05);
  }

  if (millis() - timeStart < 2500) {
    if (millis() - timeStart < 1000) {
      for (uint8_t i = 0; i < 2; i++) {
        BusChain::selectBus(encoderBuses[i]);
        encoders[i].calibrate();
      }
    } else if (millis() - timeStart < 2000) {
      update();
    } else {
      ModulatedServo::drive(0);
    }
    return false;
  } else {
    for (uint8_t i = 0; i < 2; i++) {
      BusChain::selectBus(encoderBuses[i]);
      encoders[i].reset();
    }
    mode = POWER;
    return true;
  }
}
void DRIFTMotor::update() {
  for (uint8_t i = 0; i < 2; i++) {
    BusChain::selectBus(encoderBuses[i]);
    encoders[i].update();
  }
  if (mode == FORCE || mode == DISPLACEMENT) {
    separation = encoders[0].relativePosition() - encoders[1].relativePosition();
    if (mode == DISPLACEMENT) {
      separationTarget = (distTarget+spoolOffset) - encoders[1].relativePosition();
    }
    if (separationTarget < minSep) {
      separationTarget = minSep;
    }
    pid.Compute();
    drive(power);
  }
}
void DRIFTMotor::drive(double power) {
  mode = POWER;
  this->power = power;
  ModulatedServo::drive(power);
}
void DRIFTMotor::setForceTarget(double force) {
  mode = FORCE;
  separationTarget = force - spoolOffset;
}
void DRIFTMotor::setDisplacementTarget(double target) {
  mode = DISPLACEMENT;
  distTarget = target;
}
DRIFTMotor::Mode DRIFTMotor::getMode() {
  return mode;
}
double DRIFTMotor::getPosition(uint8_t encoder) {
  return encoders[encoder].relativePosition();
}