#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);  
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1); 
#define leftSensor    5
#define centerSensor  6
#define rightSensor   7
#define echoPin       11
#define trigPin       10
const int moveSpeed = 255;
const float factor = 0.5;
const int maximumRange = 80;
const int revDelay = 500;
const int fullSpeedDistance = 70;              // ADJUSTABLE - 70/75/80
const int attackDistance = 60;                 // ADJUSTABLE - 60/70/80
const unsigned long searchTurnTime = 300;      // ADJUSTABLE - 300/500/700/900
long duration, distance;
bool opponentDetected = false;

void setup()
{
  AFMS.begin();
  pinMode(leftSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightSensor, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  delay(3000);
}

void loop() {
  int AINL = digitalRead(leftSensor);
  int AINC = digitalRead(centerSensor);
  int AINR = digitalRead(rightSensor);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2;
  if (!AINL && !AINC && !AINR) {
    avoidEdge();
  }
  else if (AINL && AINC && !AINR) {
    avoidEdge();
  }
  else if (!AINL && AINC && AINR) {
    avoidEdge();
  }
  else if (AINL && !AINC && !AINR) {
    avoidEdge();
  }
  else if (!AINL && !AINC && AINR) {
    avoidEdge();
  }
  if (distance <= maximumRange) {
    opponentDetected = true;
  } else {
    opponentDetected = false;
  } if (opponentDetected) {
    attackOpponent();
  } else {
    searchForOpponent();
  }
  delay(100);
}

void avoidEdge() {
  leftMotor->setSpeed(moveSpeed * factor);
  rightMotor->setSpeed(moveSpeed);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
  delay(revDelay);
}

void attackOpponent() {
  float attackFactor;
  if (distance <= fullSpeedDistance) {
    attackFactor = 1.0;
  } else {
    attackFactor = 1.0 - ((float)(distance - fullSpeedDistance) / (maximumRange - fullSpeedDistance)) * 0.5;
    attackFactor = constrain(attackFactor, 0.5, 1.0);
  }
  int attackSpeed = moveSpeed * attackFactor;
  leftMotor->setSpeed(attackSpeed);
  rightMotor->setSpeed(attackSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

void searchForOpponent() {
  static unsigned long lastActionTime = 0;
  static int phase = 0;
  static int moveDecision = -1;
  unsigned long currentTime = millis();

  if (moveDecision == -1) {
    moveDecision = random(0, 2);
    phase = 0;
    lastActionTime = currentTime;
  }
  if (currentTime - lastActionTime >= searchTurnTime) {
    lastActionTime = currentTime;
    phase++;
    if (moveDecision == 0) {
      if (phase == 1) {
        leftMotor->setSpeed(moveSpeed * factor);
        rightMotor->setSpeed(moveSpeed * factor);
        leftMotor->run(RELEASE);
        rightMotor->run(FORWARD);
      } else if (phase == 2) {
        leftMotor->run(FORWARD);
        rightMotor->run(RELEASE);
      } else {
        moveDecision = -1;
      }
    } else {
      if (phase == 1) {
        leftMotor->setSpeed(moveSpeed * factor);
        rightMotor->setSpeed(moveSpeed * factor);
        leftMotor->run(FORWARD);
        rightMotor->run(BACKWARD);
      } else if (phase == 2) {
        leftMotor->run(BACKWARD);
        rightMotor->run(FORWARD);
      } else {
        moveDecision = -1;
      }
    }
  }
}
