#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

// Create Motor Shield Object
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);

// Pin Definitions
#define trigPin       A4
#define echoPin       A3
#define leftSensor    5
#define centerSensor  6
#define rightSensor   7

// Speed Definitions
#define MAX_SPEED   255
#define TURN_FACTOR 0.5

// Distance Thresholds (in cm)
const int MAX_DISTANCE = 15;

long duration;
int distance;

void setup() {
  Serial.begin(9600);
  AFMS.begin();

  pinMode(leftSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightSensor, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
}

void loop() {
  // Read sensor states
  int L = digitalRead(leftSensor);
  int C = digitalRead(centerSensor);
  int R = digitalRead(rightSensor);

  // Measure distance with ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2; // Convert to cm

  // Line following logic
  if (!L && !C && !R) {
    Serial.println("000 - Reversing");
    leftMotor->setSpeed(MAX_SPEED);
    leftMotor->run(BACKWARD);
    rightMotor->setSpeed(MAX_SPEED);
    rightMotor->run(BACKWARD);
  }
  else if (L && C && !R) {
    Serial.println("110 - Turn Left");
    leftMotor->setSpeed(MAX_SPEED * TURN_FACTOR);
    leftMotor->run(BACKWARD);
    rightMotor->setSpeed(MAX_SPEED);
    rightMotor->run(RELEASE);
  }
  else if (!L && C && R) {
    Serial.println("011 - Turn Right");
    leftMotor->setSpeed(MAX_SPEED);
    leftMotor->run(RELEASE);
    rightMotor->setSpeed(MAX_SPEED * TURN_FACTOR);
    rightMotor->run(BACKWARD);
  }
  else if (L && !C && !R) {
    Serial.println("100 - Hard Left");
    leftMotor->setSpeed(MAX_SPEED * TURN_FACTOR);
    leftMotor->run(BACKWARD);
    rightMotor->setSpeed(MAX_SPEED);
    rightMotor->run(BACKWARD);
  }
  else if (!L && !C && R) {
    Serial.println("001 - Hard Right");
    leftMotor->setSpeed(MAX_SPEED);
    leftMotor->run(BACKWARD);
    rightMotor->setSpeed(MAX_SPEED * TURN_FACTOR);
    rightMotor->run(BACKWARD);
  }
  else if (L && C && R) {
    // Obstacle logic
    if (distance <= MAX_DISTANCE) {
      Serial.print("Obstacle Detected: ");
      Serial.println(distance);
      leftMotor->setSpeed(MAX_SPEED);
      leftMotor->run(FORWARD);
      rightMotor->setSpeed(MAX_SPEED);
      rightMotor->run(FORWARD);
    } else {
      Serial.println("Path Clear - Spinning Left");
      leftMotor->setSpeed(MAX_SPEED * TURN_FACTOR);
      leftMotor->run(BACKWARD);
      rightMotor->setSpeed(MAX_SPEED * TURN_FACTOR);
      rightMotor->run(FORWARD);
    }
  } else {
    // Default fallback
    Serial.println("Unrecognized pattern - Stopping");
    leftMotor->run(RELEASE);
    rightMotor->run(RELEASE);
  }

  delay(50);
}
