#include <DRIFTMotor.h>
#include <ServoController.h>
#include <BusChain.h>
#include <math.h>
#include <ArduinoEigenDense.h>

#define SER 4
#define CLK 15
#define RCLK 2

#define interruptPin 5
#define servoDriverPort 11

using namespace Eigen;

const uint8_t servoChannels[3] = {0, 1, 14};
const uint8_t encoderPorts[3][2] = {{14, 15}, {7, 6}, {8, 9}};

DRIFTMotor motors[3];

uint64_t startTime;

//Positions of DRIFT motor outlets
Vector3f h1, h2, h3;

//Finger cap radius
float capRadius = 18.822;

void setup() {
  Serial.begin(115200);
  while (!Serial) {

  }

  //Initializes DRIFT motor outlet points (x, y, z)
  h1 << 87.21284, 36.20728, 0;
  h2 << -12.25, -93.63217, 0;
  h3 << -74.96284, 57.4249, 0;
  
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

  //Homing procedure
  for (uint8_t i = 0; i < 3; i++) {
    motors[i].beginHome();
  }
  startTime = millis();
  while (millis() - startTime < 20000) {
    for (uint8_t i = 0; i < 3; i++) {
      if (ServoController::checkPulseFlag()) {
        for (uint8_t i = 0; i < 3; i++) {
          motors[i].updateEncoders();
          motors[i].updateMPC();
        }
      }
      motors[i].updateEncoders();
    }
  }
  for (uint8_t i = 0; i < 3; i++) {
    motors[i].endHome();
  }
}

void updateMotors() {
  for (uint8_t i = 0; i < 3; i++) {
    motors[i].updateEncoders();
    motors[i].updateMPC();
  }
}

String toString(const Eigen::VectorXf &mat){
    std::stringstream ss;
    ss << mat;
    return ss.str().c_str();
}

void localize() {
  Vector3f v1, v2, Xn, Yn, Zn, s;
  float r1, r2, r3, i, d, j, x, y, z, z2;

  r1 = motors[0].getPosition() + capRadius;
  r2 = motors[1].getPosition() + capRadius;
  r3 = motors[2].getPosition() + capRadius;

  v1 = h2-h1;
  v2 = h3-h1;

  Xn = v1.normalized();
  Zn = v1.cross(v2).normalized();
  Yn = Xn.cross(Zn);

  i = Xn.dot(v2);
  d = Xn.dot(v1);
  j = Yn.dot(v2);

  x = (pow(r1, 2)-pow(r2, 2)+pow(d, 2))/(2*d);
  y = (pow(r1, 2)-pow(r3, 2)+pow(i, 2)+pow(j, 2))/(2*j) - i/j*x;
  z = sqrt(max(0., pow(r1, 2)-pow(x, 2)-pow(y,2)));

  s = h1 + x*Xn + y*Yn + z*Zn;

  Serial.println(toString(s));
  Serial.println();
}

void loop() {
  for (uint8_t i = 0; i < 3; i++) {
    if (ServoController::checkPulseFlag()) {
      updateMotors();
    }
    motors[i].updateEncoders();
    localize();
  }
  Serial.println();
}