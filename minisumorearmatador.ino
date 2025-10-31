#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);
#define position1 145
#define position2 50
Servo leftServo;
Servo rightServo;

#define echoPin       9
#define trigPin       8
#define leftSensor    5
#define centerSensor  6
#define rightSensor   7
#define forward   255
#define reverse   255
#define moveSpeed 255
#define factor    0.5
#define revDelay  500

int maximumRange = 80;
long duration, distance;

void setup()
{
  Serial.begin(9600);
  AFMS.begin();

  pinMode(leftSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightSensor, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  leftServo.attach(10);
  rightServo.attach(9);
  leftServo.write(position1);
  rightServo.write(position2);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  delay(3000);
  leftServo.write(position2);
  rightServo.write(position1);
  leftMotor->setSpeed(moveSpeed);
  rightMotor->setSpeed(moveSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  delay(500);
}

void loop()
{
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

  if (!AINL && !AINC && !AINR)
  {
    Serial.println("000");
    leftMotor->setSpeed(moveSpeed);
    rightMotor->setSpeed(moveSpeed);
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    delay(revDelay);
  }
  else if (AINL && AINC && !AINR)
  {
    Serial.println("110");
    leftMotor->setSpeed(moveSpeed * factor);
    rightMotor->setSpeed(moveSpeed);
    leftMotor->run(RELEASE);
    rightMotor->run(BACKWARD);
    delay(revDelay);
  }
  else if (!AINL && AINC && AINR)
  {
    Serial.println("011");
    leftMotor->setSpeed(moveSpeed);
    rightMotor->setSpeed(moveSpeed * factor);
    leftMotor->run(BACKWARD);
    rightMotor->run(RELEASE);
    delay(revDelay);
  }
  else if (AINL && !AINC && !AINR)
  {
    Serial.println("100");
    leftMotor->setSpeed(moveSpeed * factor);
    rightMotor->setSpeed(moveSpeed);
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    delay(revDelay);
  }
  else if (!AINL && !AINC && AINR)
  {
    Serial.println("001");
    leftMotor->setSpeed(moveSpeed);
    rightMotor->setSpeed(moveSpeed * factor);
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    delay(revDelay);
  }
  else if (AINL && AINC && AINR)
  {
    if (distance <= maximumRange)
    {
      Serial.println("111 " + String(distance) + " ATTACK");
      leftMotor->setSpeed(moveSpeed);
      rightMotor->setSpeed(moveSpeed);
      leftMotor->run(FORWARD);
      rightMotor->run(FORWARD);
    }
    else
    {
      Serial.println("111 OOR");
      leftMotor->setSpeed(moveSpeed * factor);
      rightMotor->setSpeed(moveSpeed * factor);
      leftMotor->run(BACKWARD);
      rightMotor->run(FORWARD);
    }
    delay(100);
  }
}
