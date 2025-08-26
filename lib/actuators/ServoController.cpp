/*
  ServoController.cpp - Uses PCA9685 PWM controller and meta-PWM for precise control of servos
  Created by Carson G. Ray
*/

#include "ServoController.h"

const uint16_t ServoController::pwmFreq = 50;
const uint32_t ServoController::oscillatorFreq = 28300000;
const uint16_t ServoController::rangeCenter = 308;
const uint16_t ServoController::deadZone = 6;
const uint16_t ServoController::rangeLength = 102;
const uint16_t ServoController::deadband = 1;
const uint16_t ServoController::deadbandRes = 20;
const uint16_t ServoController::pwmRangeLength = 4096;

ServoController::ServoController()
    : driverChannel(0), busChain(nullptr)
{
	//Dynamic memory allocation for spinlock
	spinlock = (portMUX_TYPE*) malloc(sizeof(portMUX_TYPE));
	// Initialize the spinlock dynamically
	portMUX_INITIALIZE(spinlock);

    reset();
}

/// @brief Initializes ServoController object
/// @param driverChannel Channel number of servo driver on BusChain
/// @param busChain BusChain object
/// @return true (successful), false (error)
bool ServoController::begin(uint8_t driverChannel, BusChain* busChain) {
    // Initializes communication parameters
    this->driverChannel = driverChannel;
    this->busChain = busChain;

	// Ensures busChain is enabled
	busChainEnable = true;

    // Selects busChain channel and initializes sensor settings
    busChain->selectChannel(driverChannel);
    bool ret = begin(busChain->getI2CBus());
    busChain->release();

    // Returns success
    return ret;
}

/// @brief Initializes ServoController object
/// @param wire I2CBus object
/// @return true (successful), false (error)
bool ServoController::begin(TwoWire* wire) {
    pwmDriver = Adafruit_PWMServoDriver(DEFAULT_ADDRESS, *wire);
    bool ret = pwmDriver.begin();
    pwmDriver.setOscillatorFrequency(oscillatorFreq);
    pwmDriver.setPWMFreq(pwmFreq);
    // Uses first channel to trigger interrupt
    pwmDriver.setPWM(0, 0, 100);

    // Resets pwm parameters
    reset();

    // Returns success
    return ret;
}

/// @brief Resets all servo parameters
void ServoController::reset() {
    taskENTER_CRITICAL(spinlock);
    for (uint8_t i = 0; i < MAX_SERVOS; i++) {
        pwmSignals[i] = 0;
    }
    taskEXIT_CRITICAL(spinlock);
}

/// @brief Sets signal of servo
/// @param channel Servo channel number
/// @param power Signal value (-1 to 1)
void ServoController::setSignal(uint8_t channel, float power) {
    // Clamps power to range
    if (power > 1) {
        power = 1;
    } else if (power < -1) {
        power = -1;
    }
    // Determines direction of power
    int8_t dir = (int8_t) (abs(power)/power);
    // Scales power to range
    int16_t scaled = round(power*rangeLength);
    // Determines intervals of deadband width and base PWM
    uint16_t pwmSignal = rangeCenter + deadZone*dir + scaled;

    // Updates servo parameters
    taskENTER_CRITICAL_ISR(spinlock);
    pwmSignals[channel] = pwmSignal; 
    taskEXIT_CRITICAL_ISR(spinlock);
}

/// @brief Updates PWM driver with new PWM ranges
void ServoController::updatePWMDriver(uint8_t channel) {
    if (busChainEnable) {
        busChain->selectChannel(driverChannel);
    }
    // Computes spreading of PWM spikes to prevent unnecessary current draw
    uint16_t maxSignal = rangeCenter + deadZone + rangeLength;
    uint16_t pwmStart = (maxSignal*channel)%pwmRangeLength;
    pwmDriver.setPWM(channel + 1, pwmStart, pwmStart + pwmSignals[channel]);
    if (busChainEnable) {
        busChain->release();
    }
}