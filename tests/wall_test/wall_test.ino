#include <TrackRing.h>
#include <BusChain.h>
#include <math.h>
#include <ESP32Servo.h>

#define SER 4
#define CLK 15
#define RCLK 2

#define servoPin 5

const uint16_t encoderPorts[2] = {0, 1};

Servo servo;

const float unitsPerRadian = 1/PI;
const int8_t encoderDirs[2] = {-1, 1};

const float wallDist = -12;
const float spoolOffset = 4.5;
const float minSep = 2.25;
const float p = -0.2;
const float i = 0;
const float iCap = 0.1;
const float d = 0;

// TrackRing objects
TrackRing encoders[2];

uint32_t clockSpeed = 400000;

unsigned long timeStart = 0;

void driveServo(float power) {
  if (power > 1000) {
    power = 1000;
  } else if (power < -1000) {
    power = -1000;
  }
  servo.writeMicroseconds(1500+power);
}

float prevError = 0;
float sumError = 0;

float pid(float error) {
  float pError = error;
  float iError = 0;
  float dError = 0;

  if (timeStart != 0) {
    float stepTime = timeStart - micros();
    if (abs(error) < iCap) {
      sumError += error*stepTime;
      iError = sumError;
    } else {
      sumError = 0;
    }
    dError = (error-prevError)/stepTime;
  }
  
  timeStart = micros();
  prevError = error;

  return p*pError + i*iError + d*dError;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {

  }
  
  // Allocates specific timer
	ESP32PWM::allocateTimer(1);
	servo.attach(servoPin);
  
  BusChain::begin(SER, CLK, RCLK, 1, clockSpeed);
  for (uint8_t i = 0; i < 2; i++) {
    uint8_t err = BusChain::selectPort(encoderPorts[i]);
    if (err != 0) {
      Serial.print("Error selecting I2C port: ");
      Serial.println(encoderPorts[i]);
      while (true) {
        ;
      }
    }
    
    if (encoders[i].begin()) {
      Serial.print("Error initializing encoder: ");
      Serial.println(encoderPorts[i]);
      while (true) {
          ;
      }
    }
    encoders[i].setUnitsPerRadian(unitsPerRadian);
    encoders[i].setDirection(encoderDirs[i]);
  }
  
  Serial.println("Force control test");

  driveServo(-50);
  timeStart = millis();
  while (millis() - timeStart < 2000) {
    if (millis() - timeStart < 1000) {
      for (uint8_t i = 0; i < 2; i++) {
        BusChain::selectPort(encoderPorts[i]);
        encoders[i].calibrate();
      }
    } else {
      for (uint8_t i = 0; i < 2; i++) {
        BusChain::selectPort(encoderPorts[i]);
        encoders[i].update();
      }
    }
  }
  driveServo(0);
  delay(500);
  for (uint8_t i = 0; i < 2; i++) {
    BusChain::selectPort(encoderPorts[i]);
    encoders[i].reset();
  }
  timeStart = 0;
}

void loop() {
  for (uint8_t i = 0; i < 2; i++) {
    BusChain::selectPort(encoderPorts[i]);
    encoders[i].update();
  }
  float separation = encoders[0].relativePosition() - encoders[1].relativePosition();
  float separationTarget = (wallDist+spoolOffset) - encoders[1].relativePosition();
  if (separationTarget < minSep) {
    separationTarget = minSep;
  }
  driveServo(pid(separation - separationTarget));
  Serial.println(encoders[1].relativePosition());
}