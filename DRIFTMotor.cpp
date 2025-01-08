/**
 * DRIFTMotor.cpp - Dynamic resistance integrated force-feedback and tracking motor
 * Created by Carson G. Ray
*/

#include "DRIFTMotor.h"

/// @brief Default constructor
DRIFTMotor::DRIFTMotor() {
	// Initialize the spinlock dynamically
	portMUX_INITIALIZE(spinlock);
}

/// @brief Assigns servo and encoders to motor
/// @param servoChannel ServoController channel
/// @param encoderPort0 BusChain port for servo encoder
/// @param encoderPort1 BusChain port for spool encoder
/// @return -1 (successful), >-1 (encoder port where error occured)
int16_t DRIFTMotor::attach(uint8_t servoChannel, uint8_t encoderPort0, uint8_t encoderPort1) {
  	this->servoChannel = servoChannel;

	for (uint8_t i = 0; i < 2; i++) {
		//Assigns encoder ports
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

		//Opens BusChain channel and connects to encoder
		if (BusChain::selectPort(port) != 0) {
			return port;
		}
		if (!encoders[i].begin()) {
			return port;
		}

		//Sets parameters
		encoders[i].setUnitsPerRadian(unitsPerRadian);
		encoders[i].setDirection(encoderDirs[i]);
	}
	return -1;
}

/// @brief Reads magnetic sensor data
void DRIFTMotor::updateSensors() {
	for (uint8_t i = 0; i < 2; i++) {
		BusChain::selectPort(encoderPorts[i]);
		encoders[i].updateData();
	}
}

/// @brief Interpolates encoder positions
void DRIFTMotor::updateEncoders() {
	for (uint8_t i = 0; i < 2; i++) {
		BusChain::selectPort(encoderPorts[i]);
		encoders[i].updatePosition();
	}
	if (getMode() == HOMING) {
		if (getEncoderPos(1) < homePos) {
			homePos = getEncoderPos(1);
		}
	}
}

/// @brief Resets both motor encoders
void DRIFTMotor::resetEncoders() {
	for (uint8_t i = 0; i < 2; i++) {
		BusChain::selectPort(encoderPorts[i]);
		encoders[i].reset();
	}
}

/// @brief Updates servo PID controller
void DRIFTMotor::updateMPC() {
	//Updates sampled encoder velocities
	taskENTER_CRITICAL(spinlock);
	for (uint8_t i = 0; i < 2; i++) {
		velocities[i] = encoders[i].sampledVelocity();
	}
	taskEXIT_CRITICAL(spinlock);

	//PID cannot be updated during calibration
	Mode currMode = getMode();
	if (currMode == HOMING || currMode == FORCE || currMode == DISPLACEMENT) {
		//Gets predicted position of spool encoder at the end of the horizon time
		float predictedPos = getPredEncoderPos(1);

		taskENTER_CRITICAL(spinlock);
		if (currMode == DISPLACEMENT) {
			//Separation target is set to enforce desired displacement
			separationTarget = predictedPos - (distTarget-spoolOffset);
		}
		if (separationTarget < minSep) {
			//A minimum separation prevents string from becoming slack
			separationTarget = minSep;
		}
		taskEXIT_CRITICAL(spinlock);

		//Gets necessary spool velocity to reach separation target
		float necessaryVel = ((predictedPos-separationTarget)-getEncoderPos(0))/(horizonTime/1000000.);
		
		//Sets power based on necessary velocity
		setPower(necessaryVel*velocityCorrelation);
	}
}

/// @brief Sets servo power
/// @param power + (unspooling), - (spooling)
void DRIFTMotor::setPower(float power) {
  	ServoController::setPower(servoChannel, power*servoDir);
}

/// @brief Sets motor force applied
/// @param force distance tortional spring is engaged
void DRIFTMotor::setForceTarget(float force) {
  setMode(FORCE);
  taskENTER_CRITICAL(spinlock);
  if (force > 0) {
    separationTarget = spoolOffset + force;
  } else {
	//If force is zero, no need to be right on the cusp of the tortional spring
    separationTarget = minSep;
  }
  taskEXIT_CRITICAL(spinlock);
}

/// @brief Sets spool displacement limit
/// @param target displacement limit
void DRIFTMotor::setDisplacementTarget(float target) {
  setMode(DISPLACEMENT);
  taskENTER_CRITICAL(spinlock);
  distTarget = target+homePos;
  taskEXIT_CRITICAL(spinlock);
}

/// @brief Gets current mode
/// @return mode enum
DRIFTMotor::Mode DRIFTMotor::getMode() {
	taskENTER_CRITICAL(spinlock);
	Mode currMode = mode;
	taskEXIT_CRITICAL(spinlock);
  	return currMode;
}

/// @brief Sets mode
/// @param mode mode enum
void DRIFTMotor::setMode(Mode mode) {
	taskENTER_CRITICAL(spinlock);
  	this->mode = mode;
	taskEXIT_CRITICAL(spinlock);
  	if (getMode() == HOMING) {
		setForceTarget(0);
  	}
}

void DRIFTMotor::beginHoming() {
	setMode(HOMING);
}

void DRIFTMotor::endHoming() {
	setMode(FORCE);
}

uint8_t DRIFTMotor::getChannel() {
	return servoChannel;
}

/// @brief Gets the position of an encoder
/// @param encoder 0 (servo), 1 (spool)
/// @return encoder position
float DRIFTMotor::getEncoderPos(uint8_t encoder) {
	return encoders[encoder].relativePosition();
}
/// @brief Gets the position of the motor after homing
/// @return position
float DRIFTMotor::getPosition() {
	taskENTER_CRITICAL(spinlock);
	float home = homePos;
	taskEXIT_CRITICAL(spinlock);
  return getEncoderPos(1) - home;
}

/// @brief Gets next predicted position of spool after horizon time
/// @return position
float DRIFTMotor::getPredEncoderPos(uint8_t encoder) {
	return getEncoderPos(encoder) + getEncoderVel(encoder)*horizonTime/1000000.;
}

float DRIFTMotor::getPredictedPos() {
	return getPredEncoderPos(1) - homePos;
}

/// @brief Gets the velocity of an encoder
/// @param encoder 0 (servo encoder), 1 (spool encoder)
/// @return velocity in units per second
float DRIFTMotor::getEncoderVel(uint8_t encoder) {
	taskENTER_CRITICAL(spinlock);
	float vel = velocities[encoder];
	taskEXIT_CRITICAL(spinlock);
  	return velocities[encoder];
}
/// @brief Gets the velocity of the motor spool
/// @return velocity
float DRIFTMotor::getVelocity() {
	return getEncoderVel(1);
}

/// @brief Gets separation between spool and servo encoders
/// @return separation
float DRIFTMotor::getSeparation() {
  return getEncoderPos(1) - getEncoderPos(0);
}