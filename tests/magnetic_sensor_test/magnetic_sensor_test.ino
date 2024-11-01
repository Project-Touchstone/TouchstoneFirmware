/*****************************************************************************

 *
 * Copyright (C) 2023 Infineon Technologies AG (INFINEON). All rights reserved.
 *
 * THIS SOFTWARE IS PROVIDED "AS IS".  NO WARRANTIES, WHETHER EXPRESS, IMPLIED
 * OR STATUTORY, INCLUDING, BUT NOT LIMITED TO, IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE APPLY TO THIS SOFTWARE.
 * INFINEON SHALL NOT, IN ANY CIRCUMSTANCES, BE LIABLE FOR DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES, FOR ANY REASON WHATSOEVER.

 *
 ******************************************************************************/
/* ---------------------------------------------------------------- */

/* Very basic sketch to read out data from the TLI493D-W2BW sensor  */
/* with the master controlled 1-byte-read mode triggering ADC on    */
/* read before first MSB                                            */
/* ---------------------------------------------------------------- */

#include <Wire.h>       // default I²C library
#define ADDRESS 0x5E
#define DELAY 500       // value in ms, change to modify update rate
void setup() {
  Serial.begin(115200);
  while (!Serial);

  // Init I2C peripheral
  Wire.begin();
  //Wire.setClock(400000);
  // Configure sensor
  Wire.beginTransmission(ADDRESS); // Sensor address
  Wire.write(0x10);             // Register address

  Wire.write(0b00010001);       // Config register: measure temperature, measure Bz, trigger ADC on read before MSB(2bits), full range, no temperature compensation(2bits), odd CP parity

  Wire.write(0b10010001);       // MOD1 register: odd Fuse parity, I2C address A0, 1-byte-read mode, no collision avoidance, enabled interrupt, master controlled mode

  int err = Wire.endTransmission();
  if (err != 0) {
    Serial.print("Error initializing sensor: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  Serial.println("Bx, By, Bz, T");
  delay(3000);
}

void loop() {

  // Readout the first 7 data bytes

  uint8_t buf[7];
  int num = Wire.requestFrom(ADDRESS, 7);
  if (num == 0) {
    Serial.print("Error reading data");
    while (true) {
      ;
    }
  }
  for (uint8_t i = 0; i < 7; i++) {
    buf[i] = Wire.read();
  }

  // Built 12 bit data

  int16_t X = (int16_t)((buf[0] << 8 )| (buf[4] & 0xF0)) >> 4;
  int16_t Y = (int16_t)((buf[1] << 8 ) | ((buf[4] & 0x0F) << 4)) >> 4;
  int16_t Z = (int16_t)((buf[2] << 8 ) | ((buf[5] & 0x0F) << 4)) >> 4;
  uint16_t T = (buf[3] << 4) | (buf[5] >> 4);

  /* --------------------------------------------------------- */

  /* Enter your application code here*/

  /* --------------------------------------------------------- */

  // Send via serial port to be displayed on a terminal

  Serial.print(X);
  Serial.print(", ");
  Serial.print(Y);
  Serial.print(", ");
  Serial.print(Z);
  Serial.print(", ");
  Serial.print(T);
  Serial.println();
  delay(DELAY);

}
