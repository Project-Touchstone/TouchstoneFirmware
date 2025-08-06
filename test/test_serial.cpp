#include <Arduino.h>
#include <unity.h>
#include "comms/SerialInterface.h"

void test_serial_begin() {
    SerialInterface::begin(115200);
    // No assertion, just check for crash
}

void test_serial_send_byte() {
    SerialInterface::sendByte(0xAA);
    // No assertion, just check for crash
}

void test_serial_send_int16() {
    SerialInterface::sendInt16(12345);
    // No assertion, just check for crash
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_serial_begin);
    RUN_TEST(test_serial_send_byte);
    RUN_TEST(test_serial_send_int16);
    UNITY_END();
}

void loop() {
    // empty
}
