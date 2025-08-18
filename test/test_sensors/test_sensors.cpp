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
    // Initialize I2C bus
    I2C.begin(I2C0_SDA, I2C0_SCL);

    // Initializes buschain object
    #ifdef BUSCHAIN_ENABLE
    busChain.begin(busChainIDs, &I2C);
    #endif
}

void test_mag_sensor_begin() {
    // Loops through all magnetic sensors in configuration and tests one at a time
    for (uint8_t i = 0; i < NUM_MAG_SENSORS; i++) {
        MagSensor mag;
        uint8_t channel = encoderChannels[i / 2][i % 2];
        TEST_ASSERT_MESSAGE(mag.begin(channel, &busChain), ("MagSensor begin failed on encoder channel " + String(channel)).c_str());
    }
}

void test_mag_tracker_begin() {
    // Loops through all magnetic trackers in configuration and tests one at a time
    for (uint8_t i = 0; i < 2; i++) {
        MagSensor tracker;
        uint8_t channel = magTrackerPorts[i];
        TEST_ASSERT_MESSAGE(tracker.begin(channel, &busChain), ("MagTracker begin failed on tracker channel " + String(channel)).c_str());
    }  
}

void test_imu_begin() {
    IMU imu;
    TEST_ASSERT_MESSAGE(imu.begin(imuChannel, &busChain), ("IMU begin failed on channel " + String(imuChannel)).c_str());
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
