// External imports
#include <Arduino.h>
#include <unity.h>

// Configuration imports
#include "HydraFOCConfig.h"
#include "../TestMapConfig.hpp"

// Internal library imports
#include "DynamicConfig.h"
#include "MagSensor.h"
#include "IMU.h"
#include "BusChain.h"

// Dynamic configuration object
DynamicConfig config;

// TwoWire objects (supporting up to 2 buses)
TwoWire I2CBuses[2] = {TwoWire(0), TwoWire(1)};

// BusChain objects (supporting up to 2 buschains)
BusChain busChains[2];

// Vectors for sensor objects
std::vector<MagSensor> magEncoders;
std::vector<MagSensor> magTrackers;
std::vector<IMU> imus;

void test_mag_encoder_begin() {
    // Loops through all magnetic encoders in configuration and tests one at a time
    for (uint8_t i = 0; i < config.numMagEncoders(); i++) {
        DynamicConfig::I2CDeviceConfig i2cConfig = config.getMagEncoder(i);
        MagSensor ms;
        magEncoders.push_back(ms);
        bool ret = config.beginI2CDevice(i2cConfig, magEncoders.back());
        TEST_ASSERT_MESSAGE(ret, ("Magnetic Encoder begin failed " + config.describeI2CDevice(i2cConfig)).c_str());
    }
}

void test_mag_tracker_begin() {
    // Loops through all magnetic trackers in configuration and tests one at a time
    for (uint8_t i = 0; i < config.numMagTrackers(); i++) {
        DynamicConfig::I2CDeviceConfig i2cConfig = config.getMagTracker(i);
        MagSensor ms;
        magTrackers.push_back(ms);
        bool ret = config.beginI2CDevice(i2cConfig, magTrackers.back());
        TEST_ASSERT_MESSAGE(ret, ("Magnetic Tracker begin failed " + config.describeI2CDevice(i2cConfig)).c_str());
    }
}

void test_imu_begin() {
    // Loops through all IMUs in configuration and tests one at a time
    for (uint8_t i = 0; i < config.numIMUs(); i++) {
        DynamicConfig::I2CDeviceConfig i2cConfig = config.getIMU(i);
        IMU imu;
        imus.push_back(imu);
        bool ret = config.beginI2CDevice(i2cConfig, imus.back());
        TEST_ASSERT_MESSAGE(ret, ("IMU begin failed " + config.describeI2CDevice(i2cConfig)).c_str());
    }
}

void setup() {
    // Set up configuration
    config.setI2CBuses(I2CBuses);
    config.setBusChains(busChains);

    // Load configuration from test file
    loadConfig(config);

    // Initialize buschain objects
    for (uint8_t i = 0; i < config.numBusChains(); i++) {
        config.beginBusChain(config.getBusChain(i), busChains[i]);
    }

    UNITY_BEGIN();
    RUN_TEST(test_mag_encoder_begin);
    RUN_TEST(test_mag_tracker_begin);
    RUN_TEST(test_imu_begin);
    UNITY_END();
}

void loop() {
    // empty
}