#include <DRIFTMotor.h>
#include <ServoController.h>
#include <BusChain.h>
#include <math.h>

#define SER 4
#define CLK 15
#define RCLK 2

#define servoChannel 0
#define interruptPin 5

#define servoDriverPort 11

const uint16_t encoderPorts[2] = {7, 6};

const float criticalPoints[3] = {5, 5.25, 5.5};
const float steepness = 3;

DRIFTMotor motor;

void setup() {
  Serial.begin(115200);
  while (!Serial) {

  }
  
  BusChain::begin(SER, CLK, RCLK, 2);
  if (!ServoController::begin(servoDriverPort, interruptPin)) {
    Serial.println("Error connecting to servo driver");
    while (true) {
      
    }
  }
  int16_t err = motor.attach(servoChannel, encoderPorts[0], encoderPorts[1]);
  if (err > -1) {
    Serial.print("Error connecting to encoder port ");
    Serial.println(err);
    while (true) {

    }
  }
  
  Serial.println("DRIFT Motor Test");

  while (!motor.calibrate()) {

  }
}

void loop() {
  if (ServoController::checkPulseFlag()) {
    if (motor.getPosition(1) < criticalPoints[0]) {
      motor.setDisplacementTarget(criticalPoints[0]);
    } else if (motor.getPosition(1) < criticalPoints[1]) {
      motor.setForceTarget((motor.getPosition(1)-criticalPoints[0])*steepness);
    } else {
      motor.setDisplacementTarget(criticalPoints[2]);
    }
    motor.updatePID();
    Serial.print("Pos: ");
    Serial.print(motor.getPosition(1));
    Serial.print("\tSep: ");
    Serial.println(motor.getSeparation());
  } else {
    motor.updateEncoders();
  }
}