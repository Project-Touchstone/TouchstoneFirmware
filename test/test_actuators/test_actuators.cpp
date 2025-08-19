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
    // Initialize I2C bus
	I2C.begin(I2C0_SDA, I2C0_SCL);

	//Initializes buschain object
    #ifdef BUSCHAIN_ENABLE
	busChain.begin(busChainIDs, &I2C);
    #endif
}

#ifdef SERVO_ENABLE
void test_servo_begin() {
    TEST_ASSERT_MESSAGE(servoController.begin(servoDriverChannel, &busChain), ("Servo controller begin failed on channel " + String(servoDriverPort)).c_str());
}

void test_servo_set_power() {
    test_servo_begin();
    // Loops through servos in configuration and tests forward, backward, and stopping one at a time
    for (uint8_t i = 0; i < NUM_SERVOS; i++) {
        servoController.setSignal(servoChannels[i], 0.1f);
        servoController.updatePWMDriver(servoChannels[i]);
        delay(1000);
        servoController.setSignal(servoChannels[i], -0.1f);
        servoController.updatePWMDriver(servoChannels[i]);
        delay(1000);
        servoController.setSignal(servoChannels[i], 0.0f);
        servoController.updatePWMDriver(servoChannels[i]);
        delay(1000);
    }
    // No assertion, just check whether each servo turns on
}
#endif

void setup() {
    UNITY_BEGIN();
    #ifdef SERVO_ENABLE
    RUN_TEST(test_servo_begin);
    RUN_TEST(test_servo_set_power);
    #endif
    UNITY_END();
}

void loop() {
    // empty
}
