#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

// Motor shield setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);  // Left motor
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // Right motor

// Servo setup for flags
Servo leftServo;
Servo rightServo;
#define position1 145
#define position2 50

// Pin definitions for sensors
#define leftSensor    5
#define centerSensor  6
#define rightSensor   7
#define echoPin       9
#define trigPin       8

// Constants
const int moveSpeed = 255;
const float factor = 0.5;
const int maximumRange = 80;
const int fullSpeedDistance = 20;
const int revDelay = 500;
const int attackDistance = 15;

long duration, distance;
bool opponentDetected = false;

void setup() {
  Serial.begin(9600);
  AFMS.begin();

  pinMode(leftSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightSensor, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);

  // Attach servos
  leftServo.attach(10);
  rightServo.attach(9);
  leftServo.write(position1);  // Default inward
  rightServo.write(position2);

  delay(3000);  // Initial delay

  // Set initial motor speeds
  leftMotor->setSpeed(moveSpeed * factor);
  rightMotor->setSpeed(moveSpeed * factor);
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

  opponentDetected = (distance <= maximumRange);

  // Edge detection logic
  if (!AINL && !AINC && !AINR) {
    Serial.println("Edge detected: 000");
    avoidEdge();
  } else if (AINL && AINC && !AINR) {
    Serial.println("Edge detected: 110");
    avoidEdge();
  } else if (!AINL && AINC && AINR) {
    Serial.println("Edge detected: 011");
    avoidEdge();
  } else if (AINL && !AINC && !AINR) {
    Serial.println("Edge detected: 100");
    avoidEdge();
  } else if (!AINL && !AINC && AINR) {
    Serial.println("Edge detected: 001");
    avoidEdge();
  }

  // Opponent behavior
  if (opponentDetected) {
    attackOpponent();
  } else {
    searchForOpponent();
  }

  delay(100);
}

void avoidEdge() {
  leftMotor->setSpeed(moveSpeed);
  rightMotor->setSpeed(moveSpeed);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
  delay(revDelay);

  leftMotor->setSpeed(moveSpeed * factor);
  rightMotor->setSpeed(moveSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
  delay(500);
}

void attackOpponent() {
  Serial.println("ATTACK - Opponent detected");

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
  Serial.println("No opponent detected - Searching...");

  int moveDecision = random(0, 2);

  if (moveDecision == 0) {
    // Zigzag left (back left, forward right)
    leftMotor->setSpeed(moveSpeed * factor);
    rightMotor->setSpeed(moveSpeed * factor);
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);

    // Swoosh right flag (as we’re turning left)
    rightServo.write(position1);
    delay(50);
    rightServo.write(position2);
    delay(50);

    delay(500);

    // Zigzag right (forward left, back right)
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);

    // Swoosh left flag (as we’re turning right)
    leftServo.write(position2);
    delay(50);
    leftServo.write(position1);
    delay(50);

    delay(500);
  } else {
    // Spin in place (left forward, right backward)
    leftMotor->setSpeed(moveSpeed * factor);
    rightMotor->setSpeed(moveSpeed * factor);
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);

    // Swoosh both flags
    leftServo.write(position2);
    rightServo.write(position1);
    delay(50);
    leftServo.write(position1);
    rightServo.write(position2);
    delay(50);

    delay(500);

    // Spin the other way
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);

    // Swoosh both again
    leftServo.write(position2);
    rightServo.write(position1);
    delay(50);
    leftServo.write(position1);
    rightServo.write(position2);
    delay(50);

    delay(500);
  }
}
