#include "HydraFOCMotor.h"

HydraFOCMotor::HydraFOCMotor(int pwmA, int pwmB, int pwmC, int enPin)
    : motor(7), // 7 pole pairs as example, adjust as needed
      driver(pwmA, pwmB, pwmC, enPin),
      enablePin(enPin),
      targetVelocity(0),
      targetPosition(0),
      targetVoltage(0),
      mode(VELOCITY)
{
}

void HydraFOCMotor::begin() {
    driver.voltage_power_supply = 12; // Set supply voltage as needed
    driver.init();
    motor.linkDriver(&driver);
    motor.controller = MotionControlType::velocity_openloop;
    motor.init();
    motor.initFOC();
    pinMode(enablePin, OUTPUT);
    digitalWrite(enablePin, HIGH);
}

void HydraFOCMotor::setVelocity(float velocity) {
    targetVelocity = velocity;
    mode = VELOCITY;
    motor.controller = MotionControlType::velocity_openloop;
}

void HydraFOCMotor::setPosition(float position) {
    targetPosition = position;
    mode = POSITION;
    motor.controller = MotionControlType::angle_openloop;
}

void HydraFOCMotor::setVoltage(float voltage) {
    targetVoltage = voltage;
    mode = VOLTAGE;
    motor.controller = MotionControlType::torque;
}

void HydraFOCMotor::update() {
    switch (mode) {
        case VELOCITY:
            motor.move(targetVelocity);
            break;
        case POSITION:
            motor.move(targetPosition);
            break;
        case VOLTAGE:
            motor.move(targetVoltage);
            break;
    }
}

float HydraFOCMotor::getPosition() const {
    return motor.shaft_angle;
}

float HydraFOCMotor::getVelocity() const {
    return motor.shaft_velocity;
}