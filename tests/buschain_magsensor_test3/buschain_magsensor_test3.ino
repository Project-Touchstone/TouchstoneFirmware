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

uint8_t buffer[2];

void convertToBytes(uint16_t num) {
  buffer[0] = (uint8_t) (num >> 8);
  buffer[1] = (uint8_t) (num & 0b00001111);
}

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
  //Serial.println("3D Magnetic Sensor Test");
}

//long cycleStart;

void loop() {
  //cycleStart = micros();
 delay(delaytime); // wait time between reads.

  if (TLV493D.update(sensorAddr) == 0) {
        convertToBytes(TLV493D.convertToMag(TLV493D.rawX()));
        Serial.write(buffer, 2);
        convertToBytes(TLV493D.convertToMag(TLV493D.rawY()));
        Serial.write(buffer, 2);
        convertToBytes(TLV493D.convertToMag(TLV493D.rawZ()));
        Serial.write(buffer, 2);
        Serial.write('\n');
  } else {
    //Serial.println("Data read error!");
  }
  //Serial.println(micros()-cycleStart);
}