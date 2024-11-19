/*
  ModulatedServo.h - Increases speed resolution of continuous servo with modulated PWM signal
  Created by Carson G. Ray
*/

#ifndef ModulatedServo_h
#define ModulatedServo_h

#include "Arduino.h"
#include <ESP32Servo.h>
#include <math.h>

class ModulatedServo {
    private:
        static Servo servo;
        static int8_t direction;
        static uint32_t servoInterval;
        static uint32_t rangeCenter;
        static uint32_t deadZone;
        static uint32_t rangeLength;
        static uint16_t rangeInterval;
        
        static uint32_t pwmLevel[2];
        static uint16_t criticalCount;
        static volatile uint16_t pulseCount;

        static hw_timer_t* timer;
        static portMUX_TYPE timerMux;

        static void IRAM_ATTR onTimer();
    public:
        static void attach(uint8_t pin);
        static void setDirection(int8_t direction);
        static void drive(float power);
};

#endif