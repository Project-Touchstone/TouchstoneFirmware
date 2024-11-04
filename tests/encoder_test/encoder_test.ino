#include <Tlv493d.h>
#include <BusChain.h>
#include <math.h>

#define SER 3
#define CLK 4
#define RCLK 5

#define targetBus 7

// Tlv493d Opject
Tlv493d magSensor = Tlv493d();

uint16_t clockSpeed = 400000;

float amplitudes[3] = {0, 0, 0};

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  BusChain.begin(SER, CLK, RCLK, 1, clockSpeed);

  int err = BusChain.selectBus(targetBus);
  if (err != 0) {
    Serial.print("Error selecting I2C Bus: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  magSensor.begin();
  magSensor.setAccessMode(magSensor.MASTERCONTROLLEDMODE);
  magSensor.disableTemp();
  Serial.println("Magnetic Encoder Test");
}

void loop() {
  magSensor.updateData();
  bool trigger = false;
  if (abs(magSensor.getX()) > amplitudes[0]) {
    amplitudes[0] = abs(magSensor.getX());
    trigger = true;
  }
  if (abs(magSensor.getY()) > amplitudes[1]) {
    amplitudes[1] = abs(magSensor.getY());
    trigger = true;
  }
  if (abs(magSensor.getZ()) > amplitudes[2]) {
    amplitudes[2] = abs(magSensor.getZ());
    trigger = true;
  }
  if (trigger) {
    for (int i = 0; i < 3; i++) {
        Serial.print(amplitudes[i]);
        Serial.print("\t");
    }
    Serial.println();
  }
}