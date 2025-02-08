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
#define DEFAULT_ADDRESS 0x40

class ServoController {
    private:
        // PWM driver object
        static Adafruit_PWMServoDriver pwmDriver;
        // Port number of servo driver on BusChain
        static uint8_t driverPort;
        // BusChain object
        static BusChain* busChain;
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
        
        // Start of PWM range for each servo
        static int16_t pwmStarts[MAX_SERVOS];
        // End of PWM range for each servo
		static uint16_t pwmEnds[MAX_SERVOS];
        // Base PWM value for each servo
        static uint16_t basePWMs[MAX_SERVOS];
        // Critical count for each servo
        static int16_t criticalCounts[MAX_SERVOS];
        // Number of pulses within meta pwm cycle
        static volatile uint8_t pulseCount;
        // Start time of pwm cycle
        static volatile uint64_t startTime;
        // Delay for I2C communication
        static uint16_t commsDelay;

        // Spinlock for RTOS
        static portMUX_TYPE spinlock;

    public:
        static bool begin(uint8_t driverPort, BusChain* busChain);
        static void reset();
        static void setPower(uint8_t channel, float power);
        static void updatePWMTime();
		static void updatePWMCompute(uint8_t channel);
        static void updatePWMDriver(uint8_t channel);
};

#endif