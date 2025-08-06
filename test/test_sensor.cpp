#include <Arduino.h>
#include <unity.h>
#include "sensors/MagSensor.h"
#include "sensors/IMU.h"
#include "comms/BusChain.h"

BusChain busChain;

void test_mag_sensor_begin() {
    MagSensor mag;
    TEST_ASSERT_TRUE(mag.begin(0, &busChain));
}

void test_mag_sensor_update() {
    MagSensor mag;
    mag.begin(0, &busChain);
    mag.update();
    // No assertion, just check for crash
}

void test_imu_begin() {
    IMU imu;
    TEST_ASSERT_TRUE(imu.begin(13, &busChain));
}

void test_imu_update() {
    IMU imu;
    imu.begin(13, &busChain);
    imu.update();
    // No assertion, just check for crash
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_mag_sensor_begin);
    RUN_TEST(test_mag_sensor_update);
    RUN_TEST(test_imu_begin);
    RUN_TEST(test_imu_update);
    UNITY_END();
}

void loop() {
    // empty
}
