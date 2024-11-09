#include <TrackRing.h>
#include <BusChain.h>
#include <math.h>

#define SER 3
#define CLK 4
#define RCLK 5

#define targetBus 7

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

  BusChain.selectBus(targetBus);
  encoder.begin();
  encoder.setPeriodsPerRev(18);
  Serial.println("Servo Control Test");

  while (encoder.calibrate()) {

  }
}

void loop() {
  encoder.update();
  Serial.println(encoder.relativePosition());
}