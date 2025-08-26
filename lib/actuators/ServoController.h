/*
  ServoController.h - Uses PCA9685 PWM controller and meta-PWM for precise control of servos
  Created by Carson G. Ray
*/

#ifndef ServoController_h
#define ServoController_h

//External imports
#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
#include <math.h>
#include <freertos/FreeRTOS.h>

//Local imports
#include "BusChain.h"
#include "I2CDevice.h"

#define MAX_SERVOS 15
#define DEFAULT_ADDRESS 0x40

class ServoController : public I2CDevice {
    private:
        // PWM driver object
        Adafruit_PWMServoDriver pwmDriver;
        // Whether BusChain is enabled
        bool busChainEnable = false;
        // Channel number of servo driver on BusChain
        uint8_t driverChannel = 0;
        // BusChain object
        BusChain* busChain;
        // PWM frequency
        static const uint16_t pwmFreq;
        // Oscillator frequency
        static const uint32_t oscillatorFreq;
        // Center of servo range
        static const uint16_t rangeCenter;
        // Dead zone of servo range
        static const uint16_t deadZone;
        // Length of servo range from center
        static const uint16_t rangeLength;
        // Deadband width
        static const uint16_t deadband;
        // Desired resolution within deadband
        static const uint16_t deadbandRes;
        // Length of PWM range
        static const uint16_t pwmRangeLength;
        
        // PWM signal for each servo
        int16_t pwmSignals[MAX_SERVOS];

        // Spinlock for RTOS
        portMUX_TYPE* spinlock;

    public:
        ServoController();
        bool begin(uint8_t driverPort, BusChain* busChain) override;
        bool begin(TwoWire* wire) override;
        void reset();
        void setSignal(uint8_t channel, float power);
        void updatePWMDriver(uint8_t channel);
};

#endif