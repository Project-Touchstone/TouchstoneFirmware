#ifndef WINDOW_CONFIGURATION_H
#define WINDOW_CONFIGURATION_H

//External imports
#include <Arduino.h>

// Cores to pin RTOS tasks to
uint8_t CORE_0 = 0;
uint8_t CORE_1 = 1;

// I2C pins
#define I2C_SDA 21
#define I2C_SCL 22

// PWM output pin on channel 0 of servo driver used for servo synchronization
#define interruptPin 4

// Servo driver port
#define servoDriverPort 3

// IMU port
#define imuPort 13

// IMU sensor id
#define imuID 0

// DRIFT motors to configure
#define NUM_SERVOS 4

//Serial parameters
#define SERIAL_BAUD_RATE 460800

// I2C parameters
#define I2C_BAUD_RATE 1000000
#define I2C_TIMEOUT 1000 // in milliseconds

// BusChain address identifiers
uint8_t busChainIDs[2] = {0, 1};

// Servo channels for DRIFT motors
const uint8_t servoChannels[NUM_SERVOS] = {0, 1, 2, 3};

// Servo power multiplier
const float servoPowerMultiplier = 1./32767.;

// Encoder ports on BusChain (servo, spool) per DRIFT motor
const uint8_t encoderPorts[NUM_SERVOS][2] = {{10, 11}, {0, 1}, {7, 6}, {8, 9}};

// Magnetic tracker ports
const uint8_t magTrackerPorts[2] = {14, 15};

#endif // WINDOW_CONFIGURATION_H