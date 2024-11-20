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
        static uint32_t pwmFreq;
        static uint32_t rangeCenter;
        static uint32_t deadZone;
        static uint32_t rangeLength;
        static uint16_t deadband;
        
        static uint32_t pwmStart[MAX_SERVOS];
        static volatile bool pulseFlag;
        static volatile uint16_t pulseCount;
        static volatile uint64_t startTime;
        static uint32_t commsDelay;

        static void IRAM_ATTR onPWMStart();
        static portMUX_TYPE interruptMux;

        static uint32_t microsToPWM(uint32_t micros);
    public:
        static void begin(uint8_t driverPort, uint8_t interruptPin);
        static void setPWM(uint8_t channel, uint32_t pulseLength);
        static void setPower(uint8_t channel, float power);
        static bool checkPulseFlag();
};

#endif