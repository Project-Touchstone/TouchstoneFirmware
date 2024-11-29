/*
  DRIFTMotor.h - Dynamic resistance integrated force-feedback and tracking motor
  Created by Carson G. Ray
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
        uint8_t servoChannel;
        TrackRing encoders[2];
        uint8_t encoderPorts[2];
        const float unitsPerRadian = 1/PI;
        const int8_t servoDir = -1;
        const int8_t encoderDirs[2] = {1, -1};

        enum Mode {
          PENDING,
          CALIBRATION,
          FORCE,
          DISPLACEMENT,
          MANUAL,
        };

        Mode mode;
        const float spoolOffset = 4.7;
        const float minSep = 2.5;
        float separation = 0;
        float separationTarget = 0;
        float distTarget = 0;
        const float p = -0.3;
        const float i = 0;
        const float d = -0.2;
        const float iCap = 0;
        PID pid = PID(p, i, d, iCap);
        
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
        float getPosition(uint8_t encoder);
        float getSeparation();
};

#endif