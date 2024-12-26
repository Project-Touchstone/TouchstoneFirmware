/**
 * DRIFTMotor.cpp - Dynamic resistance integrated force-feedback and tracking motor
 * Created by Carson G. Ray
*/

#include "DRIFTMotor.h"

/// @brief Default constructor
DRIFTMotor::DRIFTMotor() {
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

/// @brief Runs motor encoder calibration sequence (non-blocking)
/// @return true (complete), false (not complete)
bool DRIFTMotor::calibrate() {
	//If this is the first time the method has been called
	if (mode == PENDING) {
		//Starts timer and begins unspooling motor at a slow speed
		startTime = millis();
		setPower(calibrationPower);
		//Sets mode to calibration
		mode = CALIBRATION;
	}

	if (millis() - startTime < calibrationTiming[1]) {
		//If within calibration time
		if (millis() - startTime < calibrationTiming[0]) {
			//If within active calibration time
			//Updates encoders to calibrate
			updateEncoders();
		} else {
			//Updates encoders, but with the servo powered off
			updateEncoders();
			setPower(0);
		}
		return false;
	} else if (mode == CALIBRATION) {
		//Resets encoders to zero
		for (uint8_t i = 0; i < 2; i++) {
			BusChain::selectPort(encoderPorts[i]);
			encoders[i].reset();
		}
		//Ends calibration mode
		mode = MANUAL;
	}
	return true;
}

/// @brief Updates both motor encoders
void DRIFTMotor::updateEncoders() {
	for (uint8_t i = 0; i < 2; i++) {
		BusChain::selectPort(encoderPorts[i]);
		encoders[i].update();
	}
}

/// @brief Updates servo PID controller
void DRIFTMotor::updateMPC() {
	//Updates sampled encoder velocities
	for (uint8_t i = 0; i < 2; i++) {
		velocities[i] = encoders[i].sampledVelocity();
	}

	//Updates motor power / velocity correlation
	if (lastPower != 0) {
		velocityCorrelation = lastPower/velocities[0];
	}

	//PID cannot be updated during calibration
	if (mode == FORCE || mode == DISPLACEMENT) {
		//Gets predicted position of spool encoder at the end of the horizon time
		float predictedPos = getPosition(1) + getVelocity(1)*horizonTime/1000000;

		//Separation is distance between spool and servo encoders
		separation = getPosition(1) - getPosition(0);
		if (mode == DISPLACEMENT) {
			//Separation target is set to enforce desired displacement
			separationTarget = predictedPos - (distTarget-spoolOffset);
		}
		if (separationTarget < minSep) {
			//A minimum separation prevents string from becoming slack
			separationTarget = minSep;
		}

		//Gets necessary spool velocity to reach separation target
		float necessaryVel = ((predictedPos-separationTarget)-getPosition(0))/(horizonTime/1000000);
		
		//Sets power based on necessary velocity
		setPower(necessaryVel*velocityCorrelation);
	}
}

/// @brief Sets servo power
/// @param power + (unspooling), - (spooling)
void DRIFTMotor::setPower(float power) {
	lastPower = power;
  	ServoController::setPower(servoChannel, power*servoDir);
}

/// @brief Sets motor force applied
/// @param force distance tortional spring is engaged
void DRIFTMotor::setForceTarget(float force) {
  mode = FORCE;
  if (force > 0) {
    separationTarget = spoolOffset + force;
  } else {
	//If force is zero, no need to be right on the cusp of the tortional spring
    separationTarget = minSep;
  }
}

/// @brief Sets spool displacement limit
/// @param target displacement limit
void DRIFTMotor::setDisplacementTarget(float target) {
  mode = DISPLACEMENT;
  distTarget = target;
}

/// @brief Gets current mode
/// @return mode enum
DRIFTMotor::Mode DRIFTMotor::getMode() {
  return mode;
}

/// @brief Sets mode
/// @param mode mode enum
void DRIFTMotor::setMode(Mode mode) {
  this->mode = mode;
}

/// @brief Gets the position of an encoder
/// @param encoder 0 (servo encoder), 1 (spool encoder)
/// @return position
float DRIFTMotor::getPosition(uint8_t encoder) {
  return encoders[encoder].relativePosition();
}
/// @brief Gets the velocity of an encoder
/// @param encoder 0 (servo encoder), 1 (spool encoder)
/// @return velocity in units per second
float DRIFTMotor::getVelocity(uint8_t encoder) {
  return velocities[encoder];
}

/// @brief Gets separation between spool and servo encoders
/// @return separation
float DRIFTMotor::getSeparation() {
  return separation;
}