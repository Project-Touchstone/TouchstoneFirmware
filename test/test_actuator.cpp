#include <Arduino.h>
#include <unity.h>
#include "actuators/ServoController.h"
#include "comms/BusChain.h"

BusChain busChain;

void test_servo_begin() {
    TEST_ASSERT_TRUE(ServoController::begin(3, &busChain));
}

void test_servo_set_power() {
    ServoController::setPower(0, 0.5f);
    // No assertion, just check for crash
}

void test_servo_reset() {
    ServoController::reset();
    // No assertion, just check for crash
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_servo_begin);
    RUN_TEST(test_servo_set_power);
    RUN_TEST(test_servo_reset);
    UNITY_END();
}

void loop() {
    // empty
}
