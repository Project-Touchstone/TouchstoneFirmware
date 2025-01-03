/*
  ServoController.h - Uses PCA9685 PWM controller and meta-PWM for precise control of servos
  Created by Carson G. Ray
*/

#ifndef ServoController_h
#define ServoController_h

#include "Arduino.h"
#include <BusChain.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>

#define MAX_SERVOS 16

class ServoController {
    private:
        static Adafruit_PWMServoDriver pwmDriver;
        static uint8_t driverPort;
        static const uint16_t pwmFreq;
        static const uint32_t oscillatorFreq;
        static const uint16_t rangeCenter;
        static const uint16_t deadZone;
        static const uint16_t rangeLength;
        static const uint16_t deadband;
        static const uint16_t deadbandRes;
        
        static int16_t pwmStart[MAX_SERVOS];
        static int16_t pwmEnd[MAX_SERVOS];
        static volatile uint8_t pulseCount;
        static volatile bool pulseFlag;
        static volatile uint64_t startTime;
        static uint16_t commsDelay;

        static void IRAM_ATTR onPWMStart();
        static portMUX_TYPE interruptMux;

    public:
        static bool begin(uint8_t driverPort, uint8_t interruptPin);
        static void reset();
        static void setPWM(uint8_t channel, uint16_t pulseLength);
        static void setPower(uint8_t channel, float power);
        static bool checkPulseFlag();
};

#endif