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
        const float unitsPerRadian = 26/12;
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

        //Operating mode
        enum Mode {
          CALIBRATION,
		  HOMING,
          FORCE,
          DISPLACEMENT,
          MANUAL,
        };

		//Default mode is pending
        Mode mode = CALIBRATION;
		//Distance between spool clutch and servo clutch
        const float spoolOffset = 14.63;
		//Minimal separation between spool and servo encoders
        const float minSep = 7.57;
		//Target separation between encoders
        float separationTarget = 0;
		//Distance target for spool encoder
        float distTarget = 0;

		//Predicted position
		float predictedPos = 0;
		//Velocity correlation for model predictive control
		const float velocityCorrelation = 0.0065;
		//Horizon time for model predictive control
		const uint32_t horizonTime = 20000;
        
		//Calibration timer start
        uint64_t startTime = 0;

		//Home position
		float homePos = 0;

        //Spinlock for RTOS
        portMUX_TYPE* spinlock;

		float getEncoderPos(uint8_t encoder);
		float getEncoderVel(uint8_t encoder);
		float getPredEncoderPos(uint8_t encoder);
        void setMode(Mode mode);
    public:
        DRIFTMotor();
        int16_t attach(uint8_t servoChannel, uint8_t encoderPort0, uint8_t encoderPort1);
        void updateMPC();
        void updateSensor(uint8_t encoder);
		void updateEncoder(uint8_t encoder);
        void resetEncoders();
        void setPower(float power);
        void setForceTarget(float force);
        void setDisplacementTarget(float target);
        Mode getMode();
        void beginHoming();
        void endHoming();
        float getPosition();
		float getPredictedPos();
        float getVelocity();
        float getSeparation();
};

#endif