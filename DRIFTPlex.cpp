/**
 * DRIFTPlex.cpp - A group of DRIFT motors controlling a single node
 * Created by Carson G. Ray
*/

#include "DRIFTPlex.h"

void DRIFTPlex::attach(DRIFTMotor* motors, Vector3f* homePoints, uint8_t numMotors) {
    this->motors = motors;
    this->homePoints = homePoints;
    this->numMotors = numMotors;
}

void DRIFTPlex::localize() {
  Vector3f v1, v2, Xn, Yn, Zn, s;
  float r1, r2, r3, i, d, j, x, y, z, z2;

  r1 = motors[0].getPosition();
  r2 = motors[1].getPosition();
  r3 = motors[2].getPosition();

  v1 = homePoints[1]-homePoints[0];
  v2 = homePoints[2]-homePoints[0];

  Xn = v1.normalized();
  Zn = v1.cross(v2).normalized();
  Yn = Xn.cross(Zn);

  i = Xn.dot(v2);
  d = Xn.dot(v1);
  j = Yn.dot(v2);

  x = (pow(r1, 2)-pow(r2, 2)+pow(d, 2))/(2*d);
  y = (pow(r1, 2)-pow(r3, 2)+pow(i, 2)+pow(j, 2))/(2*j) - i/j*x;
  z = sqrt(max(0., pow(r1, 2)-pow(x, 2)-pow(y,2)));

  position = homePoints[0] + x*Xn + y*Yn + z*Zn;
  
  Vector3f slantVel;
  for (int i = 0; i < numMotors; i++) {
    slantVel(i) = motors[i].getVelocity();
  }
  
  for (int i = 0; i < numMotors; i++) {
    slants(i, all) = (position - homePoints[i]).normalized();
  }
  Vector3f velocity = slants.bdcSvd(ComputeThinU | ComputeThinV).solve(slantVel);
}

void DRIFTPlex::setForceTarget() {
    Vector3f force;
    force << 0, 0, 0;
    setForceTarget(force);
}

void DRIFTPlex::setForceTarget(Vector3f force) {
    setMode(FORCE);
    this->forceTarget = force;
}

void DRIFTPlex::setPositionLimit(Vector3f target, bool collision) {
    setMode(POSITION);
    this->posLimit = posLimit;
    this->collision = collision;
}

void DRIFTPlex::updateController() {
    switch(getMode()) {
        case FORCE:
            for (int i = 0; i < numMotors; i++) {
                motors[i].setForceTarget(forceTarget.dot(slants(i, all)));
            }
            break;
        case POSITION:
            for (int i = 0; i < numMotors; i++) {
                float currPos = motors[i].getPosition();
                float newPos = (posLimit - homePoints[i]).norm();
                if (collision != (newPos > currPos)) {
                    motors[i].setPositionLimit(newPos);
                } else {
                    motors[i].setForceTarget(0);
                }
            }
            break;
    }
}

void DRIFTPlex::setMode(Mode mode) {
    this->mode = mode;
}

DRIFTPlex::Mode DRIFTPlex::getMode() {
    return mode;
}

Vector3f DRIFTPlex::getPosition() {
    return position;
}

Vector3f DRIFTPlex::getVelocity() {
    return velocity;
}

Vector3f DRIFTPlex::getPredictedPos() {
    return getPosition() + getVelocity()*DRIFTMotor::getHorizonTime();
}

float DRIFTPlex::getPredictedPos(uint8_t motor) {
    return motors[motor].getPosition() + getVelocity().dot(slants(motor, all))*DRIFTMotor::getHorizonTime();
}