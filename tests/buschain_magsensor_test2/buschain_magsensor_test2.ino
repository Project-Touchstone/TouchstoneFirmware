#include <TLV493D.h>
#include <BusChain.h>

#define SER 3
#define CLK 4
#define RCLK 5

#define targetBus 7

byte delaytime = 1;

const uint16_t sensorAddr = 0x5E;
bool speed = true;

uint16_t clockSpeed = 400000;

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
  TLV493D.begin(sensorAddr, 1);
  Serial.println("3D Magnetic Sensor Test");
}

//long cycleStart;

void loop() {
  //cycleStart = micros();
 delay(delaytime); // wait time between reads.

  if (TLV493D.update(sensorAddr) == 0) {
        Serial.print(TLV493D.convertToMag(TLV493D.getX()));
        Serial.print("\t");
        Serial.print(TLV493D.convertToMag(TLV493D.getY()));
        Serial.print("\t");
        Serial.print(TLV493D.convertToMag(TLV493D.getZ()));
        Serial.print("\t");
        Serial.println(TLV493D.convertToCelsius(TLV493D.getT()));
  } else {
    Serial.println("Data read error!");
  }
  //Serial.println(micros()-cycleStart);
}