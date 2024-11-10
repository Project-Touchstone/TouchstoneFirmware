#include <TrackRing.h>
#include <BusChain.h>
#include <math.h>
#include <ESP32Servo.h>

#define SER 3
#define CLK 4
#define RCLK 5

#define servoPin 6

#define targetBus 7

Servo servo;

const float unitsPerRadian = 1/PI;

// TrackRing object
TrackRing encoder = TrackRing();

uint16_t clockSpeed = 400000;

unsigned long timeStart = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  // Allocates specific timer
	ESP32PWM::allocateTimer(1);
	servo.attach(servoPin); // attaches the servo on pin 18 to the servo object
  
  BusChain::begin(SER, CLK, RCLK, 1, clockSpeed);
  uint8_t err = BusChain::selectBus(targetBus);
  if (err != 0) {
    Serial.print("Error selecting I2C Bus: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  
  if (!encoder.begin()) {
    Serial.println("Error initializing encoder");
    while (true) {
        ;
    }
  }
  encoder.setUnitsPerRadian(unitsPerRadian);
  Serial.println("Servo Control Test");

  servo.write(89);
  timeStart = millis();
  while (millis() - timeStart < 4000) {
    encoder.calibrate();
  }
  encoder.reset();
  servo.write(90);
}

void loop() {
  encoder.update();
  Serial.println(encoder.absolutePosition());
}