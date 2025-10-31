#include <Wire.h>
#include <Servo.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *leftMotor = AFMS.getMotor(4);
Adafruit_DCMotor *rightMotor = AFMS.getMotor(1);
Servo leftServo;
Servo rightServo;
#define echoPin       4
#define trigPin       3
#define leftSensor    5
#define centerSensor  6
#define rightSensor   7
#define forward       255
#define reverse       255
#define moveSpeed     255
#define factor        0.5
#define revDelay      500
#define lposition2    150
#define rposition2    150
#define initialDelay  3000        //5000 for competition
#define angleSetDelay 200         //200 for side set   | 300 for wide set
#define distSetDelay  1500*1.0     //1.0 - far side set | 0.5 - near side set
#define motorSpin1    FORWARD     //switch to change direction
#define motorSpin2    BACKWARD    //switch to change direction
#define motorTactic1  FORWARD     //BACKWARD | RELEASE  | FORWARD
#define motorTactic2  FORWARD    //RELEASE  | BACKWARD | FORWARD
int maximumRange = 80;            //100 - qualifying | 80 - for actual
long duration, distance;

void setup()
{
  //Serial.begin(9600);
  AFMS.begin();
  pinMode(leftSensor, INPUT);
  pinMode(centerSensor, INPUT);
  pinMode(rightSensor, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  leftMotor->run(RELEASE);
  rightMotor->run(RELEASE);
  delay(initialDelay);
  rightServo.attach(9); 
  leftServo.attach(10);
  leftServo.write(lposition2);
  rightServo.write(rposition2);
  leftMotor->setSpeed(moveSpeed * factor);
  rightMotor->setSpeed(moveSpeed * factor);
  leftMotor->run(motorSpin1);                                                                               
  rightMotor->run(motorSpin2);                                                                              
  delay(angleSetDelay);
  leftMotor->setSpeed(moveSpeed);
  rightMotor->setSpeed(moveSpeed);
  leftMotor->run(FORWARD);
  rightMotor->run(FORWARD);
  delay(distSetDelay);               
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

  if (!AINL && !AINC && !AINR) // 000
  {
    //Serial.println("000");
    leftMotor->setSpeed(moveSpeed);
    rightMotor->setSpeed(moveSpeed);
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    delay(revDelay);
  }
  else if (AINL && AINC && !AINR) //110
  {
    //Serial.println("110");
    leftMotor->setSpeed(moveSpeed * factor);
    rightMotor->setSpeed(moveSpeed);
    leftMotor->run(RELEASE);
    rightMotor->run(BACKWARD);
    delay(revDelay);
  }
  else if (!AINL && AINC && AINR) //011
  {
    //Serial.println("011");
    leftMotor->setSpeed(moveSpeed);
    rightMotor->setSpeed(moveSpeed * factor);
    leftMotor->run(BACKWARD);
    rightMotor->run(RELEASE);
    delay(revDelay);
  }
  else if (AINL && !AINC && !AINR) //100
  {
    //Serial.println("100");
    leftMotor->setSpeed(moveSpeed * factor);
    rightMotor->setSpeed(moveSpeed);
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    delay(revDelay);
  }
  else if (!AINL && !AINC && AINR) //001
  {
    //Serial.println("001");
    leftMotor->setSpeed(moveSpeed);
    rightMotor->setSpeed(moveSpeed * factor);
    leftMotor->run(BACKWARD);
    rightMotor->run(BACKWARD);
    delay(revDelay);
  }
  else if (AINL && AINC && AINR) //111
  {
    if (distance <= maximumRange)
    {
      //Serial.println("111 " + String(distance) + " ATTACK");
      leftMotor->setSpeed(moveSpeed);
      rightMotor->setSpeed(moveSpeed);
      leftMotor->run(motorTactic1);                                                                            
      rightMotor->run(motorTactic2);                                                                           
      delay(300);
    }
    else
    {
      //Serial.println("111 OOR");
      leftMotor->setSpeed(moveSpeed * factor);  
      rightMotor->setSpeed(moveSpeed * factor); 
      leftMotor->run(motorSpin2);                                                                       
      rightMotor->run(motorSpin1);                                                                           
    }
    delay(100);
  }
}
