#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *right = AFMS.getMotor(2);
Adafruit_DCMotor *left = AFMS.getMotor(3);

#define trigPin      A3
#define echoPin      A4
#define leftSensor   A2
#define centerSensor A1
#define rightSensor  A0
#define forward     255
#define reverse     255
#define turn        255
#define factor      0.5


int maximumRange = 15;
int minimumRange = 0;
long duration, distance;
int AINL = digitalRead(leftSensor);
int AINC = digitalRead(centerSensor);
int AINR = digitalRead(rightSensor);

void setup()
{
  AFMS.begin();
  Serial.begin(9600);
  pinMode(leftSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightSensor, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

}

void loop()
{
  digitalWrite(8, HIGH);
  digitalWrite(12, HIGH);
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = duration / 58.2;

  if (!AINL && !AINC && !AINR) // 000
  {
    Serial.println("000");
    left->setSpeed(reverse);
    left->run(BACKWARD);
    right->setSpeed(reverse);
    right->run(BACKWARD);
  }
  else if (AINL && AINC && !AINR) //110
  {
    Serial.println("110");
    left->setSpeed(reverse * factor);
    left->run(BACKWARD);
    right->setSpeed(reverse);
    right->run(RELEASE);
  }
  else if (!AINL && AINC && AINR) //011
  {
    Serial.println("011");
    left->setSpeed(reverse);
    left->run(RELEASE);
    right->setSpeed(reverse * factor);
    right->run(BACKWARD);
  }
  else if (AINL && !AINC && !AINR) //100
  {
    Serial.println("100");
    left->setSpeed(reverse * factor);
    left->run(BACKWARD);
    right->setSpeed(reverse);
    right->run(BACKWARD);
  }
  else if (!AINL && !AINC && AINR) //001
  {
    Serial.println("001");
    left->setSpeed(reverse);
    left->run(BACKWARD);
    right->setSpeed(reverse * factor);
    right->run(BACKWARD);
  }
  else if (AINL && AINC && AINR) //111
  {
    /*
      if ((distance <= maximumRange) && (distance >= minimumRange))
      {
      Serial.println(distance);
      left->setSpeed(forward);
      left->run(FORWARD);
      right->setSpeed(forward);
      right->run(FORWARD);
      }
      else
      {
      Serial.println("-1");
      left->setSpeed(reverse * factor);
      left->run(BACKWARD);
      right->setSpeed(forward * factor);
      right->run(FORWARD);
      //delay(500);

      left->setSpeed(forward);
      left->run(FORWARD);
      right->setSpeed(reverse);
      right->run(BACKWARD);
      delay(500);
      }
    */
  }
}
