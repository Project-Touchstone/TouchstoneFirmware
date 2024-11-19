/*
  DRIFTMotor.cpp - Dynamic resistance integrated force-feedback and tracking motor
  Created by Carson G. Ray
*/

#include "DRIFTMotor.h"

DRIFTMotor::DRIFTMotor() {
}

int16_t DRIFTMotor::attach(uint8_t servoPin, uint16_t encoderBus0, uint16_t encoderBus1) {
  ModulatedServo::attach(servoPin);
  ModulatedServo::setDirection(-1);

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

    pid.setOutputRange(0, 1);
    pid.setStepTime(20);
  }
  return -1;
}
bool DRIFTMotor::calibrate() {
  if (mode != CALIBRATION) {
    mode = CALIBRATION;
    timeStart = millis();
    ModulatedServo::drive(0.05);
  }

  bool stopped = false;
  if (millis() - timeStart < 2500) {
    if (millis() - timeStart < 1000) {
      for (uint8_t i = 0; i < 2; i++) {
        BusChain::selectBus(encoderBuses[i]);
        encoders[i].calibrate();
      }
    } else if (millis() - timeStart < 2000) {
      update();
    } else if (!stopped) {
      ModulatedServo::drive(0);
      stopped = true;
    }
    return false;
  } else {
    for (uint8_t i = 0; i < 2; i++) {
      BusChain::selectBus(encoderBuses[i]);
      encoders[i].reset();
    }
    pid.reset();
    mode = MANUAL;
    return true;
  }
}
void DRIFTMotor::update() {
  for (uint8_t i = 0; i < 2; i++) {
    BusChain::selectBus(encoderBuses[i]);
    encoders[i].update();
  }
  if (mode == FORCE || mode == DISPLACEMENT) {
    separation = encoders[1].relativePosition() - encoders[0].relativePosition();
    if (mode == DISPLACEMENT) {
      separationTarget = encoders[1].relativePosition() - (distTarget+spoolOffset);
    }
    if (separationTarget < minSep) {
      separationTarget = minSep;
    }
    drive(pid.update(separation - separationTarget));
  }
}
void DRIFTMotor::drive(float power) {
  mode = MANUAL;
  ModulatedServo::drive(power);
}
void DRIFTMotor::setForceTarget(float force) {
  mode = FORCE;
  separationTarget = spoolOffset + force;
}
void DRIFTMotor::setDisplacementTarget(float target) {
  mode = DISPLACEMENT;
  distTarget = target;
}
DRIFTMotor::Mode DRIFTMotor::getMode() {
  return mode;
}
float DRIFTMotor::getPosition(uint8_t encoder) {
  return encoders[encoder].relativePosition();
}
float DRIFTMotor::getSeparation() {
  return separation;
}