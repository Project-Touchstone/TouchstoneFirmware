#include <Wire.h>
#include <math.h>
#include <Tlv493d.h>
#include <I2C.h>

#define SER 3
#define CLK 4
#define RCLK 5

#define targetBus 7

// Variable Declaration
const byte addr = 0x5E; // default address of magnetic sensor 0x5E, 0x3E or 0X1F
byte rbuffer[10];       // store data from sensor read registers
byte delaytime = 1;     // time to wait before next read
bool PRINT_RAW_VALUES = false;

uint32_t clockInterval = 1000;

bool state = false;

uint8_t serBuffer = 0;

uint8_t serRemaining = 0;

uint32_t timerStart = 0;

uint8_t addresses[2] = {0, 1};
bool enabled[2] = {true, true};

char strBuffer[32];

int selectBus(uint8_t bus) {
  Wire.beginTransmission(0x70 + addresses[bus/8]);
  Wire.write(1 << bus%8);
  return Wire.endTransmission();
}

void setParams(uint8_t addrA, bool enA, uint8_t addrB, bool enB) {
  serBuffer = (addrB << 5) + (enB << 4) + (addrA << 1) + enA;
  serRemaining = 8;

  resetTimer();
}

void resetTimer() {
  timerStart = micros();
  state = false;
}

void disableOutput() {
  serBuffer = 0;
  serRemaining = 8;
  resetTimer();
}

void sendBit() {
  if (serRemaining > 0) {
    digitalWrite(SER, serBuffer & 1);
    serBuffer = serBuffer >> 1;
    digitalWrite(CLK, 1);
    if (serRemaining == 1) {
      digitalWrite(RCLK, 1);
    }
  }
}

void updateData() {
  uint32_t currTime = micros();
  if (currTime >= timerStart) {
    timerStart = currTime + clockInterval;
    state = !state;
    if (state) {
      sendBit();
    } else {
      if (serRemaining > 0) {
        serRemaining--;
      }
      digitalWrite(CLK, 0);
      digitalWrite(RCLK, 0);
      digitalWrite(SER, 0);
    }
  }
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {
    ;
  }

  pinMode(CLK, OUTPUT);
  pinMode(RCLK, OUTPUT);
  pinMode(SER, OUTPUT);

  disableOutput();

  while (serRemaining > 0) {
    updateData();
  }

  setParams(addresses[0], enabled[0], addresses[1], enabled[1]);

  while (serRemaining > 0) {
    updateData();
  }

  Wire.begin();
  Wire.setClock(400000);

  int err = selectBus(targetBus);
  if (err != 0) {
    Serial.print("Error selecting I2C Bus: ");
    Serial.println(err);
    while (true) {
      ;
    }
  }

  I2c.begin();
  I2c.timeOut(100);
  I2c.setSpeed(1);
  I2c.write((int) addr, 0x00,0x05);
  Serial.println("3D Magnetic Sensor Test");
}

//long cycleStart;

void loop() {
  //cycleStart = micros();
 delay(delaytime); // wait time between reads.
  // Read sensor registers and store in rbuffer
    I2c.read((int) addr,7);
      for(int i=0; i < 7; i++){
        rbuffer[i] = I2c.receive();
      }  

  // Goto decode functions below     
  int x = decodeX(rbuffer[0],rbuffer[4]);
  int y = decodeY(rbuffer[1],rbuffer[4]);
  int z = decodeZ(rbuffer[2],rbuffer[5]);
  int t = decodeT(rbuffer[3],rbuffer[6]);

  if(rbuffer[3] & B00000011 != 0){ // If bits are not 0, TLV is still reading Bx, By, Bz, or T
    Serial.println("Data read error!");

  }
  else {
    if(PRINT_RAW_VALUES){
        Serial.print(x);
        Serial.print("\t");
        Serial.print(y);
        Serial.print("\t");
        Serial.print(z);
        Serial.print("\t");
        Serial.println(t);
    }
    else{
        Serial.print(convertToMag(x));
        Serial.print("\t");
        Serial.print(convertToMag(y));
        Serial.print("\t");
        Serial.print(convertToMag(z));
        Serial.print("\t");
        Serial.println(convertToCelsius(t));
    }
  }
  //Serial.println(micros()-cycleStart);
}

//-- Begin Buffer Decode Routines --//
int decodeX(int a, int b){
/* Shift all bits of register 0 to the left 4 positions.  Bit 8 becomes bit 12.  Bits 0:3 shift in as zero.
 * Determine which of bits 4:7 of register 4 are high, shift them to the right four places -- remask in case
 * they shift in as something other than 0.  bitRead and bitWrite would be a bit more elegant in next version
 * of code.
 */
  int ans = ( a << 4 ) | (((b & B11110000) >> 4) & B00001111);

  if( ans >= 2048){ ans = ans - 4096; } // Interpret bit 12 as +/-
  return ans;
  }

int decodeY(int a, int b){
/* Shift all bits of register 1 to the left 4 positions.  Bit 8 becomes bit 12.  Bits 0-3 shift in as zero.
 * Determine which of the first four bits of register 4 are true.  Add to previous answer.
 */

  int ans = (a << 4) | (b & B00001111);
  if( ans >= 2048){ ans = ans - 4096;} // Interpret bit 12 as +/-
  return ans;
}

int decodeZ(int a, int b){
/* Shift all bits of register 2 to the left 4 positions.  Bit 8 becomes bit 12.  Bits 0-3 are zero.
 * Determine which of the first four bits of register 5 are true.  Add to previous answer.
 */
  int ans = (a << 4) | (b & B00001111);
  if( ans >= 2048){ ans = ans - 4096;}
  return ans;
}

int decodeT(int a, int b){
/* Determine which of the last 4 bits of register 3 are true.  Shift all bits of register 3 to the left 
 * 4 positions.  Bit 8 becomes bit 12.  Bits 0-3 are zero.
 * Determine which of the first four bits of register 6 are true.  Add to previous answer.
 */
  int ans;
  a &= B11110000;
  ans = (a << 4) | b;
  if( ans >= 2048){ ans -= 4096;}
  return ans;
}


float convertToMag(int a){
  return a * 0.098;
}

float convertToCelsius(int a){
  return (a-320)* 1.1;
}
