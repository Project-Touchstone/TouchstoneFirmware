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
  }

  pid.setOutputRange(-1, 1);
  return -1;
}

bool DRIFTMotor::calibrate() {
  if (mode == PENDING) {
    startTime = millis();
    setPower(0.05);
    mode = CALIBRATION;
  }

  if (millis() - startTime < calibrationTiming[2]) {
    if (millis() - startTime < calibrationTiming[0]) {
      for (uint8_t i = 0; i < 2; i++) {
        BusChain::selectPort(encoderPorts[i]);
        encoders[i].calibrate();
      }
    } else if (millis() - startTime < calibrationTiming[1]) {
      updateEncoders();
    } else {
      updateEncoders();
      setPower(0);
    }
    return false;
  } else if (mode == CALIBRATION) {
    for (uint8_t i = 0; i < 2; i++) {
      BusChain::selectPort(encoderPorts[i]);
      encoders[i].reset();
    }
    pid.reset();
    mode = MANUAL;
  }
  return true;
}
void DRIFTMotor::updateEncoders() {
  for (uint8_t i = 0; i < 2; i++) {
    BusChain::selectPort(encoderPorts[i]);
    encoders[i].update();
  }
}

void DRIFTMotor::updatePID() {
  if (mode == FORCE || mode == DISPLACEMENT) {
    separation = encoders[1].relativePosition() - encoders[0].relativePosition();
    if (mode == DISPLACEMENT) {
      separationTarget = encoders[1].relativePosition() - (distTarget-spoolOffset);
    }
    if (separationTarget < minSep) {
      separationTarget = minSep;
    }
    
    setPower(pid.update(separationTarget - separation));
  }
}

void DRIFTMotor::setPower(float power) {
  ServoController::setPower(servoChannel, power*servoDir);
}
void DRIFTMotor::setForceTarget(float force) {
  mode = FORCE;
  if (force > 0) {
    separationTarget = spoolOffset + force;
  } else {
    separationTarget = minSep;
  }
}
void DRIFTMotor::setDisplacementTarget(float target) {
  mode = DISPLACEMENT;
  distTarget = target;
}
DRIFTMotor::Mode DRIFTMotor::getMode() {
  return mode;
}
void DRIFTMotor::setMode(Mode mode) {
  this->mode = mode;
}
float DRIFTMotor::getPosition(uint8_t encoder) {
  return encoders[encoder].relativePosition();
}
float DRIFTMotor::getSeparation() {
  return separation;
}