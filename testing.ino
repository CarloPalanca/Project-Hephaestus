#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(4);
Adafruit_DCMotor *motorRight = AFMS.getMotor(1);

#define forward   255
#define turn      255
#define lSensor 5
#define cSensor 6
#define rSensor 7

void setup() {
  AFMS.begin();
  pinMode(5, INPUT);  // Left
  pinMode(6, INPUT);  // Center
  pinMode(7, INPUT);  // Right
  motorLeft->run(RELEASE);
  motorRight->run(RELEASE);
  delay(3000);
}

void loop() {
  int L = digitalRead(5);  // Left
  int C = digitalRead(6);  // Center
  int R = digitalRead(7);  // Right

  if (!L && C && !R) //010
  {
    motorLeft->setSpeed(forward);
    motorRight->setSpeed(forward);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
  }
  else if (!L && C && R) //011
  {
    motorLeft->setSpeed(forward);
    motorRight->setSpeed(forward);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
  }
  else if (L && C && !R) //110
  {
    motorLeft->setSpeed(forward);
    motorRight->setSpeed(forward);
    motorLeft->run(FORWARD);
    motorRight->run(FORWARD);
  }
  else if (!L && !C && R) //001
  {
    motorLeft->setSpeed(forward);
    motorRight->setSpeed(forward);
    motorLeft->run(FORWARD);
    motorRight->run(RELEASE);
  }
  else if (L && !C && !R) //100
  {
    motorLeft->setSpeed(forward);
    motorRight->setSpeed(forward);
    motorLeft->run(RELEASE);
    motorRight->run(FORWARD);
  }
}
