#ifndef BALANCED_FORCE_CONTROL_HPP
#define BALANCED_FORCE_CONTROL_HPP

#include <Arduino.h>
#include "HydraFOCMotor.h"
#include "HydraFOCConfig.h"
    
// Loop counter
unsigned long loopCounter = 0;

// HydraFOC motor objects
HydraFOCMotor* motors = new HydraFOCMotor[NUM_FOC_MOTORS] {
    HydraFOCMotor(focMotorPins[0][0], focMotorPins[0][1], focMotorPins[0][2], focMotorPins[0][3], focMotorPins[0][4], focMotorPins[0][5], I2C1_SDA, focCurrentPins[0][0], focCurrentPins[0][1]),
    HydraFOCMotor(focMotorPins[1][0], focMotorPins[1][1], focMotorPins[1][2], focMotorPins[1][3], focMotorPins[1][4], focMotorPins[1][5], I2C1_SDA, focCurrentPins[1][0], focCurrentPins[1][1])
};

// Average length between motors
float averageTravelLength = 0.f;

// Whether homing is complete
bool homingComplete = false;

// Preload controller tuning
// Positive torque contracts the cable, so the controller keeps both motors
// in a small positive preload band and nudges the more-extended side harder.
float torqueBaseline = 0.25f;
float preloadKp = 0.12f;
float preloadMinTorque = 0.08f;
float preloadMaxTorque = 0.9f;

// Task function prototypes
void TaskHoming(void *pvParameters);
void TaskMotors(void *pvParameters);

// RTOS task handles
TaskHandle_t homingTaskHandle;
TaskHandle_t motorTaskHandle;

void setup() {
    Serial.begin(SERIAL_BAUD_RATE);
    // Wait for serial connection
    while (!Serial) {
        ;
    }
    // Configure I2C ports
    Wire.begin(I2C0_SDA, I2C0_SCL);
    Wire1.begin(I2C1_SDA, I2C1_SCL);

    // Configure driver pins
    pinMode(focDriverSleepPin, OUTPUT);
    pinMode(focDriverResetPin, OUTPUT);
    digitalWrite(focDriverSleepPin, HIGH); // Wake up driver
    digitalWrite(focDriverResetPin, HIGH); // Release reset

    // Initialize HydraFOC motors
    for (int i = 0; i < NUM_FOC_MOTORS; i++) {
        motors[i].begin(motorDirs[i], encoderElectricAngles[i], true, (i == 0) ? &Wire : &Wire1);
    }

    delay(1000); // Wait for motors to stabilize
    Serial.println("Balanced Force Control Initialized.");

    // RTOS task initialization
    xTaskCreatePinnedToCore(TaskHoming, "Homing Task", 4096, NULL, 2, &homingTaskHandle, CORE_0);
    xTaskCreatePinnedToCore(TaskMotors, "Motor Control Task", 4096, NULL, 1, &motorTaskHandle, CORE_0);
}

/*--------------------------------------------------*/
/*---------------------- Tasks ---------------------*/
/*--------------------------------------------------*/

void TaskHoming(void *pvParameters) {
    (void)pvParameters;
    // Saves homing locations
    float homePositions[NUM_FOC_MOTORS];

    // Saves extended positions for each motor
    float extendedPositions[NUM_FOC_MOTORS];

    // Homes motors by setting a constant force,
    // waiting for it to hit the mechanical stop, and then resetting the encoder to zero
    for (int i = 0; i < NUM_FOC_MOTORS; i++) {
        Serial.printf("Homing Motor %d\n", i);
        for (int j = 0; j < NUM_FOC_MOTORS; j++) {
            if (i == j) {
                motors[j].setTorque(3.f); // Set a moderate torque towards the stop
            } else {
                 motors[j].setTorque(0.5f); // Set enough torque to prevent slack
            }
        }
        // Wait for a short duration to allow the motor to reach the stop
        vTaskDelay(pdMS_TO_TICKS(5000));
        // Saves the homing position
        homePositions[i] = motors[i].getPosition();

        // Saves the extended position from the other motor
        int otherMotor = (i + 1) % NUM_FOC_MOTORS;
        extendedPositions[otherMotor] = motors[otherMotor].getPosition();
    }

    // Subtracts the extended positions from the home positions
    // Averages the travel length between the two motors
    for (int i = 0; i < NUM_FOC_MOTORS; i++) {
        float travelLength = abs(extendedPositions[i] - homePositions[i]);
        //Serial.printf("Motor %d Home Position: %.6f\n", i, homePositions[i]);
        //Serial.printf("Motor %d Extended Position: %.6f\n", i, extendedPositions[i]);
        //Serial.printf("Motor %d Travel Length: %.6f\n", i, travelLength);
        averageTravelLength += travelLength;

        // Resets encoder to zero at home position
        motors[i].resetEncoder(homePositions[i]);
    }
    averageTravelLength /= NUM_FOC_MOTORS;

    Serial.printf("Average Travel Length: %.6f\n", averageTravelLength);

    homingComplete = true;
    vTaskDelete(NULL); // Delete task after homing is complete
}

void TaskMotors(void *pvParameters) {
    (void)pvParameters;
    for (;;) {
        // Preload controller: keep both strings taut with a small baseline torque,
        // then correct the shared cable-length error using the homed offset.
        if (homingComplete) {
            float cableLengthError = averageTravelLength - (motors[0].getPosition() + motors[1].getPosition());
            float preloadCorrection = constrain(preloadKp * cableLengthError, -0.25f, 0.25f);

            float preloadTorque = constrain(torqueBaseline + preloadCorrection, preloadMinTorque, preloadMaxTorque);

            motors[0].setTorque(preloadTorque);
            motors[1].setTorque(preloadTorque);
        }

        // Runs FOC control loop
        for (int i = 0; i < NUM_FOC_MOTORS; i++) {
            motors[i].update();
        }

        // Motor variable monitoring
        //motor.monitor();

        // Prints current sensing readings
        /*Serial.print("Current A: ");
        Serial.print(analogRead(focCurrentPins[0][0]));
        Serial.print(" | Current B: ");
        Serial.println(analogRead(focCurrentPins[0][1]));*/

        // Prints encoder angle with full precision (every 100 loop counts)
        if (loopCounter % 100 == 0) {
            for (int i = 0; i < NUM_FOC_MOTORS; i++) {
                //Serial.print("Motor ");            Serial.print(i);
                //Serial.print(" Position: ");
                //Serial.println(motors[i].getPosition(), 6);  // 6 decimal places
            }
        }

        loopCounter++;
        taskYIELD(); // Yield to allow other tasks to run
    }
}

void loop() {
    // Empty loop since tasks are handling the control
}

#endif // BALANCED_FORCE_CONTROL_HPP