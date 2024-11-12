#include <TrackRing.h>
#include <BusChain.h>
#include <math.h>
#include <ESP32Servo.h>

#define SER 4
#define CLK 15
#define RCLK 2

#define servoPin 5

#define targetBus 7

Servo servo;

const float unitsPerRadian = 1/PI;

const float encoderTarget = 6;
const float p = -0.1;
const float i = 0;
const float d = 0;

// TrackRing object
TrackRing encoder = TrackRing();

uint16_t clockSpeed = 400000;

unsigned long timeStart = 0;

void driveServo(float power) {
  if (power > 90) {
    power = 90;
  } else if (power < -90) {
    power = -90;
  }
  servo.write(90+power);
}

float prevError = 0;
float sumError = 0;

float pid(float error) {
  float pError = error;
  float iError = 0;
  float dError = 0;

  if (timeStart != 0) {
    float stepTime = (timeStart - micros())*1000000;
    sumError += error*stepTime;
    iError = sumError;
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
  uint8_t err = BusChain::selectBus(targetBus);
  if (err != 0) {
    Serial.print("Error selecting I2C Bus: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
  if (encoder.begin()) {
    Serial.println("Error initializing encoder");
    while (true) {
        ;
    }
  }
  encoder.setUnitsPerRadian(unitsPerRadian);
  encoder.setDirection(-1);
  Serial.println("Servo Control Test");

  driveServo(1);
  timeStart = millis();
  while (millis() - timeStart < 4000) {
    if (millis() - timeStart < 3000) {
      encoder.calibrate();
    } else {
      encoder.update();
    }
  }
  driveServo(0);
  encoder.reset();
  timeStart = 0;
}

void loop() {
  encoder.update();
  driveServo(pid(encoder.relativePosition() - encoderTarget));
}