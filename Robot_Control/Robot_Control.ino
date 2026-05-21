#include <Servo.h>

Servo wristServo;
Servo clawServo;
Servo elbowServo;

int wristPos = 90;
int clawPos = 140; 
int elbowPos = 90;

void setup() {
  wristServo.attach(11);
  clawServo.attach(12);
  elbowServo.attach(13);

  wristServo.write(wristPos);
  clawServo.write(clawPos);
  elbowServo.write(elbowPos);

  Serial.begin(9600);
}

void loop() {
  if (Serial.available() >= 7) {
    byte startByte = Serial.read();

    if (startByte != 255) {
      return;
    }

    byte buffer[6];
    Serial.readBytes(buffer, 6);

    wristPos = buffer[0] | (buffer[1] << 8);
    clawPos  = buffer[2] | (buffer[3] << 8);
    elbowPos = buffer[4] | (buffer[5] << 8);

    wristPos = constrain(wristPos, 0, 180);
    clawPos  = constrain(clawPos, 0, 180);
    elbowPos = constrain(elbowPos, 0, 180);

    wristServo.write(wristPos);
    clawServo.write(clawPos);
    elbowServo.write(elbowPos);
  }
}