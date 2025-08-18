// External imports
#include <Arduino.h>
#include <unity.h>

//Configuration imports
#include "WindowConfig.h"

// Internal library imports
#include "ServoController.h"
#include "BusChain.h"

//TwoWire object
TwoWire I2C = TwoWire(0);

// BusChain object
BusChain busChain;

void setUp() {
    // Initialize I2C port
	I2C.begin(I2C_SDA, I2C_SCL);

	//Initializes buschain object
	busChain.begin(busChainIDs, &I2C);
}

void test_servo_begin() {
    TEST_ASSERT_MESSAGE(ServoController::begin(servoDriverPort, &busChain), ("Servo controller begin failed on port " + String(servoDriverPort)).c_str());
}

void test_servo_set_power() {
    test_servo_begin();
    // Loops through servos in configuration and tests forward, backward, and stopping one at a time
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        ServoController::setSignal(servoChannels[i], 0.1f);
        ServoController::updatePWMDriver(servoChannels[i]);
        delay(1000);
        ServoController::setSignal(servoChannels[i], -0.1f);
        ServoController::updatePWMDriver(servoChannels[i]);
        delay(1000);
        ServoController::setSignal(servoChannels[i], 0.0f);
        ServoController::updatePWMDriver(servoChannels[i]);
        delay(1000);
    }
    // No assertion, just check whether each servo turns on
}

void setup() {
    UNITY_BEGIN();
    RUN_TEST(test_servo_begin);
    RUN_TEST(test_servo_set_power);
    UNITY_END();
}

void loop() {
    // empty
}
