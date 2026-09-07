#ifndef FOC_MOTOR_STANDALONE_TEST_HPP
#define FOC_MOTOR_STANDALONE_TEST_HPP

#include <Arduino.h>
#include <SimpleFOC.h>
#include "HydraFOCConfig.h"

// Motor port (0 or 1)
#define MOTOR_PORT 1

// Loop counter
unsigned long loopCounter = 0;

// Motor driver
BLDCDriver3PWM driver(focMotorPins[MOTOR_PORT][0], focMotorPins[MOTOR_PORT][1], focMotorPins[MOTOR_PORT][2], 
                      focMotorPins[MOTOR_PORT][3], focMotorPins[MOTOR_PORT][4], focMotorPins[MOTOR_PORT][5]);

// Motor object
BLDCMotor motor(11);

// Magnetic encoder
MagneticSensorI2C encoder(AS5600_I2C);

// Current sense
LowsideCurrentSense currentSense(0.025f, 100.f, focCurrentPins[MOTOR_PORT][0], focCurrentPins[MOTOR_PORT][1]);

// Control mode tracking
float targetTorque = 0.005f;

int tuneCurrentController(float bandwidth) {
  // Sanity check the bandwidth
  if (bandwidth <= 0.0f) return 1; 
  
  if (motor.characteriseMotor(motor.voltage_sensor_align)) return 3;

  // Calculate PI gains based on motor parameters and desired bandwidth
  // P = L * (2 * PI * bandwidth)
  // I = R * (2 * PI * bandwidth)
  motor.PID_current_q.P = motor.phase_inductance * (_2PI * bandwidth);
  motor.PID_current_q.I = motor.phase_resistance * (_2PI * bandwidth);
  motor.PID_current_d.P = motor.phase_inductance * (_2PI * bandwidth);
  motor.PID_current_d.I = motor.phase_resistance * (_2PI * bandwidth);
  // Set current LPF time constants to cutoff at 5x bandwidth
  motor.LPF_current_d.Tf = 1.0f / (_2PI * bandwidth * 5.0f);
  motor.LPF_current_q.Tf = 1.0f / (_2PI * bandwidth * 5.0f);

  return 0;
}

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);

    if (MOTOR_PORT == 0) {
        // Configure I2C
        Wire.begin(I2C0_SDA, I2C0_SCL);
    } else if (MOTOR_PORT == 1) {
        // Configure I2C
        Wire.begin(I2C1_SDA, I2C1_SCL);
    } else {
        Serial.println("Invalid MOTOR_PORT defined. Please set to 0 or 1.");
        while (true); // Halt execution
    }

    // Configure driver pins
    pinMode(focDriverSleepPin, OUTPUT);
    pinMode(focDriverResetPin, OUTPUT);
    digitalWrite(focDriverSleepPin, HIGH); // Wake up driver
    digitalWrite(focDriverResetPin, HIGH); // Release reset

    // Initialize magnetic sensor hardware
    encoder.init();
    // Link the motor to the sensor
    motor.linkSensor(&encoder);

    // PWM frequency to be used [Hz]
    driver.pwm_frequency = 30000;
    // Power supply voltage [V]
    driver.voltage_power_supply = 12;

    driver.init();
    motor.linkDriver(&driver);

    // Maximal voltage to be set to the motor
    motor.voltage_limit = 2.8f;
    // Max current to be sent to the motor
    motor.current_limit = 1.0f;

    // Choose FOC modulation
    motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

    // Velocity PI controller parameters
    motor.PID_velocity.P = 0.2f;
    motor.PID_velocity.I = 16.f;
    motor.PID_velocity.D = 0;

    // Velocity low pass filtering time constant
    motor.LPF_velocity.Tf = 0.01f;

    // Angle P controller
    motor.P_angle.P = 20;
    // Maximal velocity of the controller
    motor.velocity_limit = 150;

    // Motor parameters
    motor.phase_resistance = 2.8; // Ohms
    motor.phase_inductance = 0.002; // Henries
    
    float bandwidth = _2PI*1.0; // Hz
    // PID tunning
    motor.PID_current_q.P = 5;//motor.phase_inductance * bandwidth;
    motor.PID_current_q.I = 10;//motor.phase_resistance * bandwidth;
    motor.PID_current_d.P = 5;//motor.phase_inductance * bandwidth;
    motor.PID_current_d.I = 10;//motor.phase_resistance * bandwidth;
    // LPF tunning
    motor.LPF_current_d.Tf = 0.1f;//1.0f / (bandwidth * 5.0f);
    motor.LPF_current_q.Tf = 0.1f;//1.0f / (bandwidth * 5.0f);

    // Enable monitoring
    motor.useMonitoring(Serial);

    // link current sense to driver and motor BEFORE motor.init()
    currentSense.linkDriver(&driver);
    // initialize current sense AFTER motor.init()
    currentSense.init();
    
    // Links current sense to driver, skips phase remapping
    currentSense.skip_align = true;
    motor.linkCurrentSense(&currentSense);

    // Initialize motor
    motor.init();

    // Align sensor and start FOC
        if (!motor.initFOC()) {
            Serial.println("FOC initialization failed; torque output disabled.");
            while (true) {
                delay(1000);
            }
        }

        /*if (!motor.pp_check_result) {
            Serial.println("Pole-pair check failed; motor characterization disabled.");
            while (true) {
                delay(1000);
            }
        }*/

        // Characterize only after initFOC has aligned the current-sense phases.
        /*float bandwidth = 30.0f; // Hz
        if (tuneCurrentController(bandwidth)) {
            Serial.println("Current-controller tuning failed; torque output disabled.");
            while (true) {
                delay(1000);
            }
        }*/

    delay(1000); // Wait for motor to stabilize
    Serial.println("FOC Motor Standalone Test Initialized.");

    // Set target torque
    motor.controller = MotionControlType::torque;
    motor.torque_controller = TorqueControlType::foc_current;
}

void loop() {
    // Run FOC control loop
    motor.loopFOC();
    motor.move(targetTorque);

    // Motor variable monitoring
    //motor.monitor();

    // Prints encoder angle with full precision (every 100 loop counts)
    if (loopCounter % 100 == 0) {
        //Serial.println(motor.shaft_angle, 6);  // 6 decimal places
    }
    
    PhaseCurrent_s currents = currentSense.getPhaseCurrents();
    float current_magnitude = currentSense.getDCCurrent();

    Serial.print(currents.a*1000); // milli Amps
    Serial.print("\t");
    Serial.print(currents.b*1000); // milli Amps
    Serial.print("\t");
    Serial.print(currents.c*1000); // milli Amps
    Serial.print("\t");
    Serial.println(current_magnitude*1000); // milli Amps

    loopCounter++;
}

#endif // FOC_MOTOR_STANDALONE_TEST_HPP
