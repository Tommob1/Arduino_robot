#include <Servo.h>

Servo servo1;
Servo servo2;
Servo servo3;

int servo1Pos;
int servo2Pos;
int servo3Pos;

int servo1Left = 10;
int servo1Right = 170;
int servo2Up = 10;
int servo2Down = 170;
int servo3Min = 10;
int servo3Max = 170;

void setup() {
  servo1.attach(11);
  servo2.attach(10);
  servo3.attach(9);
  Serial.begin(9600);
}

void loop() {
  for (servo1Pos = servo1Right; servo1Pos >= servo1Left; servo1Pos -= 1) {
    servo1.write(servo1Pos);
    delay(5);
  }

  for (servo2Pos = servo2Up; servo2Pos <= servo2Down; servo2Pos += 1) {
    servo2.write(servo2Pos);
    servo3Pos = map(servo2Pos, servo2Up, servo2Down, servo3Min, servo3Max);
    servo3.write(servo3Pos);
    delay(5);
  }

  for (servo2Pos = servo2Down; servo2Pos >= servo2Up; servo2Pos -= 1) {
    servo2.write(servo2Pos);
    servo3Pos = map(servo2Pos, servo2Down, servo2Up, servo3Max, servo3Min);
    servo3.write(servo3Pos);
    delay(5);
  }

  for (servo1Pos = servo1Left; servo1Pos <= servo1Right; servo1Pos += 1) {
    servo1.write(servo1Pos);
    delay(5);
  }

  for (servo2Pos = servo2Up; servo2Pos <= servo2Down; servo2Pos += 1) {
    servo2.write(servo2Pos);
    servo3Pos = map(servo2Pos, servo2Up, servo2Down, servo3Min, servo3Max);
    servo3.write(servo3Pos);
    delay(5);
  }

  for (servo2Pos = servo2Down; servo2Pos >= servo2Up; servo2Pos -= 1) {
    servo2.write(servo2Pos);
    servo3Pos = map(servo2Pos, servo2Down, servo2Up, servo3Max, servo3Min);
    servo3.write(servo3Pos);
    delay(5);
  }
}