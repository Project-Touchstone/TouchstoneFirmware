/**
 * DRIFTMotor.h - Dynamic resistance integrated force-feedback and tracking motor
 * Created by Carson G. Ray
*/

#ifndef DRIFTMotor_h
#define DRIFTMotor_h

#include "Arduino.h"
#include "ServoController.h"
#include <TrackRing.h>
#include <BusChain.h>
#include <math.h>

class DRIFTMotor {
    private:
		//ServoController channel
        uint8_t servoChannel;
		//Encoder objects
        TrackRing encoders[2];
		//BusChain ports for encoders
        uint8_t encoderPorts[2];
		//Units per radian
        const float unitsPerRadian = 1/PI;
		//Servo direction
        const int8_t servoDir = -1;
		//Encoder directions
        const int8_t encoderDirs[2] = {1, -1};
		//Motor power during calibration
		const float calibrationPower = 0.05;
		//First phase is active calibration, second phase is passive calibration
        const uint16_t calibrationTiming[2] = {3000, 3500};

		//Sampled velocities of encoders
        float velocities[2];
		//Last power set on servo
		float lastPower = 0;
		//Motor power per units per second velocity
		float velocityCorrelation = 0;

		//Operating mode
        enum Mode {
          PENDING,
          CALIBRATION,
          FORCE,
          DISPLACEMENT,
          MANUAL,
        };

		//Default mode is pending
        Mode mode = PENDING;
		//Distance between spool clutch and servo clutch
        const float spoolOffset = 4.85;
		//Minimal separation between spool and servo encoders
        const float minSep = 2.5;
		//Separation between spool and servo encoders
        float separation = 0;
		//Target separation between encoders
        float separationTarget = 0;
		//Distance target for spool encoder
        float distTarget = 0;

		//Horizon time for model predictive control
		uint32_t horizonTime = 20000;
        
		//Calibration timer start
        uint64_t startTime = 0;
    public:
        DRIFTMotor();
        int16_t attach(uint8_t servoChannel, uint8_t encoderPort0, uint8_t encoderPort1);
        bool calibrate();
        void updateMPC();
        void updateEncoders();
        void setPower(float power);
        void setForceTarget(float force);
        void setDisplacementTarget(float target);
        Mode getMode();
        void setMode(Mode mode);
        float getPosition(uint8_t encoder);
        float getVelocity(uint8_t encoder);
        float getSeparation();
};

#endif