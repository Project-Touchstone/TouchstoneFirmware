#include <Tlv493d.h>
#include <BusChain.h>

#define SER 3
#define CLK 4
#define RCLK 5

#define targetPort 7

// Tlv493d Opject
Tlv493d magSensor = Tlv493d();

uint32_t clockSpeed = 400000;

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }
  BusChain.begin(SER, CLK, RCLK, 1, clockSpeed);

  int err = BusChain.selectPort(targetPort);
  if (err != 0) {
    Serial.print("Error selecting I2C port: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }
  magSensor.begin();
  magSensor.setAccessMode(magSensor.MASTERCONTROLLEDMODE);
  magSensor.disableTemp();
  Serial.println("3D Magnetic Sensor Test");
}

//long cycleStart;

void loop() {
  //cycleStart = micros();
  magSensor.updateData();
  Serial.print(magSensor.getX());
  Serial.print("\t");
  Serial.print(magSensor.getY());
  Serial.print("\t");
  Serial.println(magSensor.getZ());
  //Serial.println(micros()-cycleStart);
}