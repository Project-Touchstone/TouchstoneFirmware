// External imports
#include <Arduino.h>
#include <unity.h>

//Configuration imports
#include "HydraFOCConfig.h"
#include "../TestConfigMap.hpp"

// Internal library imports
#include "DynamicConfig.h"
#include "ServoController.h"
#include "BusChain.h"

//Dynamic configuration object
DynamicConfig config;

//TwoWire object
TwoWire I2CBuses[2] = {TwoWire(0),TwoWire(1)};

// BusChain object
BusChain busChains[2];

// Vector of servo driver objects
std::vector<ServoController> servoDrivers;

void test_servo_begin() {
    // Loops through all servo drivers in configuration and tests one at a time
    for (uint8_t i = 0; i < config.numServoDrivers(); i++) {
        // Attempts to connect through associated config
        DynamicConfig::I2CDeviceConfig i2cConfig = config.getServoDriver(i);
        ServoController sc;
        servoDrivers.push_back(sc);
        bool ret = config.beginI2CDevice(i2cConfig, servoDrivers.back());
        TEST_ASSERT_MESSAGE(ret, ("Servo driver begin failed " + config.describeI2CDevice(i2cConfig)).c_str());
      }
}

void test_servo_set_power() {
    test_servo_begin();
    // Loops through servos in configuration and tests forward, backward, and stopping one at a time
    for (uint8_t i = 0; i < config.numServos(); i++) {
        DynamicConfig::ServoConfig servoConfig = config.getServo(i);
        uint8_t driverId = servoConfig.servoDriverId;
        uint8_t channel = servoConfig.channel;
        // Set forward
        servoDrivers[driverId].setSignal(channel, 0.1f);
        servoDrivers[driverId].updatePWMDriver(channel);
        delay(1000);
        // Set backward
        servoDrivers[driverId].setSignal(channel, -0.1f);
        servoDrivers[driverId].updatePWMDriver(channel);
        delay(1000);
        // Stop
        servoDrivers[driverId].setSignal(channel, 0.0f);
        servoDrivers[driverId].updatePWMDriver(channel);
        delay(1000);
    }
    // No assertion, just check whether each servo turns on
}

void setup() {
    //  Sets configuration I2C buses and buschains
    config.setI2CBuses(I2CBuses);
    config.setBusChains(busChains);

    // Loads configuration from test file
    loadConfig(config);

    //Initializes buschain objects
    for (uint8_t i = 0; i < config.numBusChains(); i++) {
        config.beginBusChain(config.getBusChain(i), busChains[i]);
    }

    UNITY_BEGIN();
    RUN_TEST(test_servo_begin);
    RUN_TEST(test_servo_set_power);
    UNITY_END();
}

void loop() {
    // empty
}