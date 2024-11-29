/*
  DRIFTMotor.cpp - Dynamic resistance integrated force-feedback and tracking motor
  Created by Carson G. Ray
*/

#include "DRIFTMotor.h"

DRIFTMotor::DRIFTMotor() {
}

int16_t DRIFTMotor::attach(uint8_t servoChannel, uint8_t encoderPort0, uint8_t encoderPort1) {
  this->servoChannel = servoChannel;

  for (uint8_t i = 0; i < 2; i++) {
    uint8_t port;
    switch(i) {
      case 0:
        port = encoderPort0;
        break;
      case 1:
        port = encoderPort1;
        break;
    }
    encoderPorts[i] = port;
    if (BusChain::selectPort(port) != 0) {
      return port;
    }
    
    if (encoders[i].begin()) {
      return port;
    }
    encoders[i].setUnitsPerRadian(unitsPerRadian);
    encoders[i].setDirection(encoderDirs[i]);

    pid.setOutputRange(0, 1);
  }
  return -1;
}
bool DRIFTMotor::calibrate() {
  if (mode != CALIBRATION) {
    startTime = millis();
    setPower(0.05);
    mode = CALIBRATION;
  }

  bool stopped = false;
  if (millis() - startTime < 2500) {
    if (millis() - startTime < 1000) {
      for (uint8_t i = 0; i < 2; i++) {
        BusChain::selectPort(encoderPorts[i]);
        encoders[i].calibrate();
      }
    } else if (millis() - startTime < 2000) {
      update();
    } else if (!stopped) {
      setPower(0);
      mode = CALIBRATION;
      stopped = true;
    }
    return false;
  } else {
    for (uint8_t i = 0; i < 2; i++) {
      BusChain::selectPort(encoderPorts[i]);
      encoders[i].reset();
    }
    pid.reset();
    ServoController::reset();
    mode = MANUAL;
    return true;
  }
}
void DRIFTMotor::update() {
  for (uint8_t i = 0; i < 2; i++) {
    BusChain::selectPort(encoderPorts[i]);
    encoders[i].update();
  }
  if (mode == FORCE || mode == DISPLACEMENT) {
    separation = encoders[1].relativePosition() - encoders[0].relativePosition();
    if (mode == DISPLACEMENT) {
      separationTarget = encoders[1].relativePosition() - (distTarget-spoolOffset);
    }
    if (separationTarget < minSep) {
      separationTarget = minSep;
    }
    setPower(pid.update(separation - separationTarget));
  }
}
void DRIFTMotor::setPower(float power) {
  mode = MANUAL;
  ServoController::setPower(servoChannel, power*servoDir);
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