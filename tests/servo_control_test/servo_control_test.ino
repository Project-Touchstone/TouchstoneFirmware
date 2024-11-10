#include <TrackRing.h>
#include <BusChain.h>
#include <math.h>

#define SER 3
#define CLK 4
#define RCLK 5

#define targetBus 7

const float unitsPerRadian = 1/PI;

// TrackRing object
TrackRing encoder = TrackRing();

uint16_t clockSpeed = 400000;

long timerStart = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  
  BusChain.begin(SER, CLK, RCLK, 1, clockSpeed);
  uint8_t err = BusChain.selectBus(targetBus);
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

  timerStart = millis();
  while (millis() - timerStart < 5000) {
    encoder.calibrateAmplitudes();
  }
  encoder.reset();
}

void loop() {
  encoder.update();
  Serial.println(encoder.absolutePosition());
}