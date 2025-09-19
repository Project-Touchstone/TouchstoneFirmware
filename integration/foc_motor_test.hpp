#ifndef FOC_MOTOR_TEST_HPP
#define FOC_MOTOR_TEST_HPP

#include <Arduino.h>
#include "HydraFOCMotor.h"
#include "HydraFOCConfig.h"

// Motor parameters
constexpr float MOTOR_POLE_PAIRS = 7;
constexpr float MOTOR_KV = 100.0f;

// HydraFOC motor object
HydraFOCMotor motor(focMotorPins[0][0], focMotorPins[0][1], focMotorPins[0][2], focMotorPins[0][3], focMotorPins[0][4], focMotorPins[0][5]);

void setup() {
    // Configure driver pins
    pinMode(focDriverSleepPin, OUTPUT);
    pinMode(focDriverResetPin, OUTPUT);
    digitalWrite(focDriverSleepPin, HIGH); // Wake up driver
    digitalWrite(focDriverResetPin, HIGH); // Release reset

    // Configure enable pins on motor A
    for (uint8_t i = 0; i < 4; i+=2) {
        pinMode(focMotorPins[0][i], OUTPUT);
        digitalWrite(focMotorPins[0][i], HIGH); // Enable mosfet
    }

    // Initialize HydraFOC motor
    motor.begin();
}

void loop() {
    // Example: Set target velocity
    motor.setVelocity(10.0f); // 10 rad/s

    // Run FOC control loop
    motor.update();
}

#endif // FOC_MOTOR_TEST_HPP