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
#include <PID.h>
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
		//First phase is active calibration, second phase is passive calibration
        const uint16_t calibrationTiming[2] = {3000, 3500};

		//Sampled velocities of encoders
        float velocities[2];

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

		//PID parameters
        const float p = -0.4;
        const float i = 0;
        const float d = -0.2;
        const float iCap = 0;
        PID pid = PID(p, i, d, iCap);
        
		//Calibration timer start
        uint64_t startTime = 0;
    public:
        DRIFTMotor();
        int16_t attach(uint8_t servoChannel, uint8_t encoderPort0, uint8_t encoderPort1);
        bool calibrate();
        void updatePID();
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