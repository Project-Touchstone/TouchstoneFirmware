/*
  DRIFTMotor.h - Dynamic resistance integrated force-feedback and tracking motor
  Created by Carson G. Ray
*/

#ifndef DRIFTMotor_h
#define DRIFTMotor_h

#include "Arduino.h"
#include "ModulatedServo.h"
#include <TrackRing.h>
#include <BusChain.h>
#include <PID_v1.h>
#include <math.h>

class DRIFTMotor {
    private:
        TrackRing encoders[2];
        uint16_t encoderBuses[2];
        const double unitsPerRadian = 1/PI;
        const int8_t encoderDirs[2] = {-1, 1};

        enum Mode {
          PENDING,
          CALIBRATION,
          FORCE,
          DISPLACEMENT,
          POWER,
        };

        Mode mode;
        const double spoolOffset = 4.5;
        const double minSep = 2.25;
        double separation = 0;
        double separationTarget = 0;
        double distTarget = 0;
        double power = 0;
        const double p = -0.2;
        const double i = 0;
        const double d = 0;
        PID pid = PID(&separation, &power, &separationTarget, p, i, d, P_ON_E, DIRECT);
        
        uint64_t timeStart = 0;
    public:
        DRIFTMotor();
        uint16_t attach(uint8_t servoPin, uint16_t encoderBus0, uint16_t encoderBus1);
        bool calibrate();
        void update();
        void drive(double power);
        void setForceTarget(double force);
        void setDisplacementTarget(double target);
        Mode getMode();
        double getPosition(uint8_t encoder);
};

#endif