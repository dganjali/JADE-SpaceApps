/*
  Simple Arduino arm controller compatible with serial lines like:
  yaw:90;p1:90;p2:45;p3:0;roll:90;cw:1\n
  Attach servos to pins configured in this file.
*/
#include <Servo.h>

Servo servoYaw;
Servo servoP1;
Servo servoP2;
Servo servoP3;
Servo servoRoll;
Servo servoClaw;

const int YAW_PIN = 6;
const int P1_PIN = 7;
const int P2_PIN = 8;
const int P3_PIN = 9;
const int ROLL_PIN = 10;
const int CLAW_PIN = 11;

float targetYaw=90, targetP1=90, targetP2=90, targetP3=90, targetRoll=0, targetClaw=90;

void setup() {
  Serial.begin(9600);
  servoYaw.attach(YAW_PIN);
  servoP1.attach(P1_PIN);
  servoP2.attach(P2_PIN);
  servoP3.attach(P3_PIN);
  servoRoll.attach(ROLL_PIN);
  servoClaw.attach(CLAW_PIN);
}

void loop() {
  if (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    int idx;
    idx = line.indexOf("yaw:");
    if (idx >= 0) targetYaw = line.substring(idx+4, line.indexOf(';', idx)).toFloat();
    idx = line.indexOf("p1:");
    if (idx >= 0) targetP1 = line.substring(idx+3, line.indexOf(';', idx)).toFloat();
    idx = line.indexOf("p2:");
    if (idx >= 0) targetP2 = line.substring(idx+3, line.indexOf(';', idx)).toFloat();
    idx = line.indexOf("p3:");
    if (idx >= 0) targetP3 = line.substring(idx+3, line.indexOf(';', idx)).toFloat();
    idx = line.indexOf("roll:");
    if (idx >= 0) targetRoll = line.substring(idx+5, line.indexOf(';', idx)).toFloat();
    idx = line.indexOf("cw:");
    if (idx >= 0) targetClaw = line.substring(idx+3, line.indexOf(';', idx)).toFloat();
  }

  // Apply to servos
  servoYaw.write((int)targetYaw);
  servoP1.write((int)targetP1);
  servoP2.write((int)targetP2);
  servoP3.write((int)targetP3);
  servoRoll.write((int)targetRoll);
  // Map claw command to servo position (0/1 -> degrees)
  int clawDeg = (int)(targetClaw > 0.5 ? 125 : 0);
  servoClaw.write(clawDeg);

  delay(20);
}
