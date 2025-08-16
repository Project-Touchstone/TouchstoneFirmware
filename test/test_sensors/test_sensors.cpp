// External imports
#include <Arduino.h>
#include <unity.h>

// Configuration imports
#include "WindowConfig.h"

// Internal library imports
#include "MagSensor.h"
#include "IMU.h"
#include "BusChain.h"

//TwoWire object
TwoWire I2C = TwoWire(0);

// BusChain object
BusChain busChain;

void setUp() {
    // Initialize I2C port
    I2C.begin(I2C_SDA, I2C_SCL);

    // Initializes buschain object
    busChain.begin(busChainIDs, &I2C);
}

void test_mag_sensor_begin() {
    // Loops through all magnetic sensors in configuration and tests one at a time
    for (uint8_t i = 0; i < NUM_SERVOS * 2; i++) {
        MagSensor mag;
        uint8_t port = encoderPorts[i / 2][i % 2];
        TEST_ASSERT_MESSAGE(mag.begin(port, &busChain), ("MagSensor begin failed on encoder port " + String(port)).c_str());
    }
}

void test_mag_tracker_begin() {
    // Loops through all magnetic trackers in configuration and tests one at a time
    for (uint8_t i = 0; i < 2; i++) {
        MagSensor tracker;
        uint8_t port = magTrackerPorts[i];
        TEST_ASSERT_MESSAGE(tracker.begin(port, &busChain), ("MagTracker begin failed on tracker port " + String(port)).c_str());
    }  
}

void test_imu_begin() {
    IMU imu;
    TEST_ASSERT_MESSAGE(imu.begin(imuPort, &busChain), ("IMU begin failed on port " + String(imuPort)).c_str());
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_mag_sensor_begin);
    RUN_TEST(test_mag_tracker_begin);
    RUN_TEST(test_imu_begin);
    UNITY_END();
}

void loop() {
    // empty
}
