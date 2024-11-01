#include <Servo.h>

//Servo pin
#define servoPin 3

#define pot A0

Servo servo;

void setup() {
  Serial.begin(9600);
  //Attaches servo
  servo.attach(servoPin);
  pinMode(pot, INPUT);
}

void loop() {
  int val = (int) analogRead(pot)*180./800;
  Serial.println(val);
  servo.write(val);
}
