//External imports
#include <Arduino.h>
#include <unity.h>

// Internal library imports
#include "SerialInterface.h"
#include "SerialHeaders.h"

using namespace SerialHeaders;

void test_serial_begin() {
    SerialInterface::begin(115200);
    // No assertion, just check for crash
}

void test_serial_send_byte() {
    SerialInterface::writeByte(0xAA);
    // No assertion, just check for crash
}

void test_serial_send_int16() {
    SerialInterface::writeInt16(12345);
    // No assertion, just check for crash
}

void test_serial_send_float() {
    SerialInterface::writeFloat(3.14f);
    // No assertion, just check for crash
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_serial_begin);
    RUN_TEST(test_serial_send_byte);
    RUN_TEST(test_serial_send_int16);
    RUN_TEST(test_serial_send_float);
    UNITY_END();
}

void loop() {
    // empty
}
