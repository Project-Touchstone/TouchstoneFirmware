#include <DRIFTMotor.h>
#include <BusChain.h>
#include <math.h>

#define SER 4
#define CLK 15
#define RCLK 2

#define servoPin 5

uint16_t encoderBuses[2] = {0, 1};
uint32_t clockSpeed = 1000000;

float criticalPoints[3] = {-5, -5.25, -5.5};
float steepness = 3;

DRIFTMotor motor;

void setup() {
  Serial.begin(115200);
  while (!Serial) {

  }
  
  BusChain::begin(SER, CLK, RCLK, 1, clockSpeed);
  int16_t err = motor.attach(servoPin, encoderBuses[0], encoderBuses[1]);
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
  motor.update();
  if (motor.getPosition(1) > criticalPoints[0]) {
    motor.setDisplacementTarget(criticalPoints[0]);
  } else if (motor.getPosition(1) > criticalPoints[1]) {
    motor.setForceTarget((criticalPoints[0]-motor.getPosition(1))*steepness);
  } else {
    motor.setDisplacementTarget(criticalPoints[2]);
  }
  /*Serial.print("Pos: ");
  Serial.print(motor.getPosition(1));
  Serial.print("\tSep: ");
  Serial.println(motor.getSeparation());*/
}