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
const float phases[3] = {0, -PI/2, -PI};
float yVals[3];
float angles[3][2];
float prevAngle = 0;
float position = 0;

long timerStart = 0;

void updateAmplitudes() {
  if (abs(magSensor.getX()) > amplitudes[0]) {
    amplitudes[0] = abs(magSensor.getX());
  }
  if (abs(magSensor.getY()) > amplitudes[1]) {
    amplitudes[1] = abs(magSensor.getY());
  }
  if (abs(magSensor.getZ()) > amplitudes[2]) {
    amplitudes[2] = abs(magSensor.getZ());
  }
}

void updateYVals() {
  yVals[0] = magSensor.getX()/amplitudes[0];
  yVals[1] = magSensor.getY()/amplitudes[1];
  yVals[2] = magSensor.getZ()/amplitudes[2];
  for (int i = 0; i < 3; i++) {
    if (yVals[i] > 1) {
      yVals[i] = 1;
    } else if (yVals[i] < -1) {
      yVals[i] = -1;
    }
  }
}

void calculateAngles() {
  for (int i = 0; i < 3; i++) {
    angles[i][0] = asin(yVals[i]);
    if (angles[i][0] >= 0) {
        angles[i][1] = PI-angles[i][0];
    } else {
        angles[i][1] = -PI-angles[i][0];
    }
    for (int j = 0; j < 2; j++) {
      angles[i][j] += phases[i];
      if (angles[i][j] > PI) {
        angles[i][j] -= 2*PI;
      } else if (angles[i][j] < -PI) {
        angles[i][j] += 2*PI;
      }
    }
  }
}

void calculatePosition() {
  float minSpread;
  uint8_t minLoc;
  for (int i = 0; i < 3; i++) {
    float spread = abs(angles[i][0] - angles[i][1]);
    if (spread > PI) {
      spread = 2*PI - spread;
    }
    if ((i == 0) || spread < minSpread) {
      minSpread = spread;
      minLoc = i;
    }
  }
  
  float minDist;
  float finalAngle;
  for (int i = 0; i < 2; i++) {
    float dist = abs(angles[minLoc][i] - prevAngle);
    if (dist > PI) {
      dist = 2*PI - dist;
    }
    if ((i == 0) || dist < minDist) {
      minDist = dist;
      finalAngle = angles[minLoc][i];
    }
  }

  float diff = finalAngle-prevAngle;
  prevAngle = finalAngle;
  position += diff;
  if (diff > PI) {
    position -= 2*PI;
  } else if (diff < -PI) {
    position += 2*PI;
  }
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
  magSensor.begin();
  magSensor.setAccessMode(magSensor.MASTERCONTROLLEDMODE);
  magSensor.disableTemp();
  Serial.println("Magnetic Encoder Test");

  timerStart = millis();
  while (millis() - timerStart < 5000) {
    magSensor.updateData();
    updateAmplitudes();
  }
  Serial.println("Encoder Calibrated");
}

void loop() {
  magSensor.updateData();
  updateYVals();
  calculateAngles();
  calculatePosition();
  Serial.println(position);
}