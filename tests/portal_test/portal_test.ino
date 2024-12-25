#include <DRIFTMotor.h>
#include <ServoController.h>
#include <BusChain.h>
#include <math.h>

#define SER 4
#define CLK 15
#define RCLK 2

#define interruptPin 5
#define servoDriverPort 11

const uint8_t servoChannels[3] = {0, 1, 14};
const uint8_t encoderPorts[3][2] = {{14, 15}, {7, 6}, {8, 9}};

DRIFTMotor motors[3];

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
  for (uint8_t i = 0; i < 3; i++) {
    int16_t err = motors[i].attach(servoChannels[i], encoderPorts[i][0], encoderPorts[i][1]);
    if (err > -1) {
      Serial.print("Error connecting to encoder port ");
      Serial.println(err);
      while (true) {

      }
    }
  }

  bool calibrated = false;
  while (!calibrated) {
    calibrated = true;
    for (uint8_t i = 0; i < 3; i++) {
      calibrated &= motors[i].calibrate();
    }
  }
  ServoController::reset();
  while (!ServoController::checkPulseFlag()) {
    
  }
  updateMotors();
}

void updateMotors() {
  for (uint8_t i = 0; i < 3; i++) {
    motors[i].updateEncoders();
    motors[i].setForceTarget(0);
    motors[i].updatePID();
  }
}

uint64_t startTime;

void loop() {
  for (uint8_t i = 0; i < 3; i++) {
    if (ServoController::checkPulseFlag()) {
      updateMotors();
    }
    motors[i].updateEncoders();
    /*Serial.print(motors[i].getPosition(1));
    Serial.print("\t");*/
  }
  //Serial.println();
}