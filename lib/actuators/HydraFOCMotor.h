#ifndef HYDRA_FOC_MOTOR_H
#define HYDRA_FCO_MOTOR_H

#include <Arduino.h>
#include <SimpleFOC.h>

// HydraFOCMotor: Wrapper for SimpleFOC motor and driver
class HydraFOCMotor {
public:
    // Construct with motor and driver pins
    HydraFOCMotor(int pwmA, int pwmB, int pwmC, int enPin);

    // Initialize the motor and driver
    void begin();

    // Set target velocity (rad/s)
    void setVelocity(float velocity);

    // Set target position (rad)
    void setPosition(float position);

    // Set voltage (open loop)
    void setVoltage(float voltage);

    // Update FOC loop (call in loop)
    void update();

    // Get current position (rad)
    float getPosition() const;

    // Get current velocity (rad/s)
    float getVelocity() const;

private:
    BLDCMotor motor;
    BLDCDriver3PWM driver;
    int enablePin;
    float targetVelocity;
    float targetPosition;
    float targetVoltage;
    enum ControlMode { VELOCITY, POSITION, VOLTAGE } mode;
};

#endif