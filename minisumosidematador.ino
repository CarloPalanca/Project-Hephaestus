#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Servo.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);

#define position1 145  // Flag idle position
#define position2 50   // Flag raised/action position
Servo leftServo;
Servo rightServo;

#define echoPin       9
#define trigPin       8

#define moveSpeed 255
#define spinSpeed 200
#define factor    0.5
#define distractionDelay 600
#define maximumRange 80

long duration, distance;

void setup()
{
  Serial.begin(9600);
  AFMS.begin();

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  leftServo.attach(10);
  rightServo.attach(9);
  leftServo.write(position1);
  rightServo.write(position2);

  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  delay(3000);
}

void loop()
{
  // Trigger ultrasonic pulse
  digitalWrite(trigPin, LOW); delayMicroseconds(2);
  digitalWrite(trigPin, HIGH); delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2;

  if (distance > 0 && distance <= maximumRange)
  {
    Serial.println("OPPONENT DETECTED: " + String(distance) + " cm");

    leftServo.write(position1);
    rightServo.write(position2);
    leftMotor->setSpeed(moveSpeed);
    rightMotor->setSpeed(moveSpeed);
    leftMotor->run(FORWARD);
    rightMotor->run(FORWARD);
    delay(distractionDelay);
  }
  else
  {
    Serial.println("SEARCHING...");

    leftServo.write(position2);
    rightServo.write(position1);
    leftMotor->setSpeed(spinSpeed);
    rightMotor->setSpeed(spinSpeed);
    leftMotor->run(FORWARD);
    rightMotor->run(BACKWARD);
  }

  delay(100);
}
