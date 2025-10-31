#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Motor shield setup
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(1);  // Left motor
Adafruit_DCMotor *rightMotor = AFMS.getMotor(2); // Right motor

// Pin definitions for sensors
#define leftSensor    5
#define centerSensor  6
#define rightSensor   7
#define echoPin       9
#define trigPin       8

// Constants
const int moveSpeed = 255;
const float factor = 0.5; 
const int maximumRange = 80;      // Max distance for detection (cm)
const int fullSpeedDistance = 20; // Full speed up to 20 cm
const int revDelay = 500;         // Delay for reversing
const int attackDistance = 15;    // Distance to start aggressive attack

// Variables for distance measurement
long duration, distance;
bool opponentDetected = false;

void setup()
{
  Serial.begin(9600);
  AFMS.begin();

  pinMode(leftSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightSensor, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  delay(3000); // Initial delay
  
  // Set initial motor speeds
  leftMotor->setSpeed(moveSpeed * factor);
  rightMotor->setSpeed(moveSpeed * factor);
}

void loop()
{
  // Read sensor inputs
  int AINL = digitalRead(leftSensor);
  int AINC = digitalRead(centerSensor);
  int AINR = digitalRead(rightSensor);

  // Measure distance with ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2;

  // Check if opponent is detected (within maximum range)
  if (distance <= maximumRange) {
    opponentDetected = true;
  } else {
    opponentDetected = false;
  }

  // Edge detection logic
  if (!AINL && !AINC && !AINR) { // 000 (edge detected)
    Serial.println("Edge detected: 000");
    avoidEdge();
  }
  else if (AINL && AINC && !AINR) { // 110
    Serial.println("Edge detected: 110 (Near left edge)");
    avoidEdge();
  }
  else if (!AINL && AINC && AINR) { // 011
    Serial.println("Edge detected: 011 (Near right edge)");
    avoidEdge();
  }
  else if (AINL && !AINC && !AINR) { // 100
    Serial.println("Edge detected: 100 (Near left edge)");
    avoidEdge();
  }
  else if (!AINL && !AINC && AINR) { // 001
    Serial.println("Edge detected: 001 (Near right edge)");
    avoidEdge();
  }

  // If the opponent is detected, initiate attack
  if (opponentDetected) {
    attackOpponent();
  } else {
    searchForOpponent();
  }

  delay(100); // Short delay to prevent constant sensor readings
}

// Function to avoid the edge and reposition
void avoidEdge() {
  leftMotor->setSpeed(moveSpeed);
  rightMotor->setSpeed(moveSpeed);
  leftMotor->run(BACKWARD);
  rightMotor->run(BACKWARD);
  delay(revDelay);

  // Smarter turn: Rotate away from the edge
  leftMotor->setSpeed(moveSpeed * factor);
  rightMotor->setSpeed(moveSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(BACKWARD);
  delay(500); // Rotate to adjust position
}

// Function to attack opponent when detected
void attackOpponent() {
  Serial.println("ATTACK - Opponent detected");

  // Adjust attack speed based on distance
  float attackFactor;
  if (distance <= fullSpeedDistance) {
    attackFactor = 1.0; // Full speed if very close
  } else {
    attackFactor = 1.0 - ((float)(distance - fullSpeedDistance) / (maximumRange - fullSpeedDistance)) * 0.5;
    attackFactor = constrain(attackFactor, 0.5, 1.0); // Limit the speed drop
  }

  int attackSpeed = moveSpeed * attackFactor;
  leftMotor->setSpeed(attackSpeed);
  rightMotor->setSpeed(attackSpeed);

  // Move forward and push the opponent
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
}

// Function to search for opponent when not detected
void searchForOpponent() {
  Serial.println("No opponent detected - Searching...");
  
  // Search Pattern (randomized behavior)
  int moveDecision = random(0, 2);
  if (moveDecision == 0) {
    // Zigzag pattern
    leftMotor->setSpeed(moveSpeed * factor);  // Slow to prevent overshooting
    rightMotor->setSpeed(moveSpeed * factor);
    leftMotor->run(BACKWARD);  // Move backward
    rightMotor->run(FORWARD);  // Move forward
    delay(500); // Delay before changing direction
    leftMotor->run(FORWARD);  // Move forward
    rightMotor->run(BACKWARD); // Move forward in opposite direction
    delay(500);
  } else {
    // Random Spin
    leftMotor->setSpeed(moveSpeed * factor);
    rightMotor->setSpeed(moveSpeed * factor);
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
    delay(500);
    leftMotor->run(BACKWARD);
    rightMotor->run(FORWARD);
    delay(500);
  }
}
