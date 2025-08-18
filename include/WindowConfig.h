#ifndef WINDOW_CONFIGURATION_H
#define WINDOW_CONFIGURATION_H

//External imports
#include <Arduino.h>

//////////////////////////////////////////////////////////////////
// RTOS configuration
//////////////////////////////////////////////////////////////////

// RTOS core affinity
uint8_t CORE_0 = 0;
uint8_t CORE_1 = 1;

////////////////////////////////////////////////////////////////////
// I2C and Serial configuration
////////////////////////////////////////////////////////////////////

// I2C pins
#define I2C0_SDA 37
#define I2C0_SCL 38
#define I2C1_SDA 39
#define I2C1_SCL 40

// I2C parameters
#define I2C_BAUD_RATE 1000000
#define I2C_TIMEOUT 1000 // in milliseconds

//Serial parameters
#define SERIAL_BAUD_RATE 460800

///////////////////////////////////////////////////////////////////
// BusChain configuration
////////////////////////////////////////////////////////////////////

//#define BUSCHAIN_ENABLE

// BusChain Wire bus
#define BUSCHAIN_WIRE_BUS 0

// BusChain address identifiers (in order of chain)
//uint8_t busChainIDs[2] = {0, 1};

////////////////////////////////////////////////////////////////////
// Servo configuration
////////////////////////////////////////////////////////////////////

//#define SERVO_ENABLE

/*
#define NUM_SERVOS 4

// PWM output pin on channel 0 of servo driver used for servo synchronization
#define interruptPin 4

// Servo driver channel on BuChain
#define servoDriverChannel 3

// Servo channels
const uint8_t servoChannels[NUM_SERVOS] = {0, 1, 2, 3};

// Servo power multiplier
const float servoSignalMultiplier = 1./32767.;*/

////////////////////////////////////////////////////////////////////
// HydraFOC motor configuration
////////////////////////////////////////////////////////////////////

#define HYDRAFOC_ENABLE

#define NUM_FOC_MOTORS 2

// HydraFOC motor pins
// Note: each motor has 3 pairs of pins, where the first is enable
// and the second is which mosfet to turn on
const uint8_t focMotorPins[NUM_FOC_MOTORS][6] =
{
    {3, 18, 9, 46, 11, 10}, // Motor 0
    {47, 21, 45, 48, 36, 35} // Motor 1
};

// Driver reset and sleep pins
#define focDriverResetPin 41
#define focDriverSleepPin 42

// Current sensing pins (2 phases per motor)
const uint8_t focCurrentPins[NUM_FOC_MOTORS][2] =
{
    {7, 6}, // Motor 0
    {17, 16} // Motor 1
};

////////////////////////////////////////////////////////////////////////////
// IMU configuration
////////////////////////////////////////////////////////////////////

//#define IMU_ENABLE

/*#define NUM_IMU 1

#define IMU_ACCEL_RANGE MPU6050_RANGE_2_G
#define IMU_GYRO_RANGE MPU6050_RANGE_250_DEG
#define IMU_FILTER_BAND MPU6050_BAND_260_HZ

// IMU buschain channels
const uint8_t imuChannels[NUM_IMU] = {13};*/

///////////////////////////////////////////////////////////////////
// Magnetic encoder configuration
////////////////////////////////////////////////////////////////////

#define MAG_ENCODER_ENABLE

#define NUM_MAG_ENCODERS NUM_FOC_MOTORS

// Magnetic encoder sensor Wire buses
// Note: order defines pairing with motors
const uint8_t magEncoderBuses[NUM_MAG_ENCODERS] = {0, 1};

//////////////////////////////////////////////////////////////
// Magnetic tracker configuration
////////////////////////////////////////////////////////////////////

//#define MAG_TRACKER_ENABLE

/*#define NUM_MAG_TRACKERS 2
// Magnetic tracker channels on BusChain
const uint8_t magTrackerChannels[2] = {14, 15};*/


#endif // WINDOW_CONFIGURATION_H