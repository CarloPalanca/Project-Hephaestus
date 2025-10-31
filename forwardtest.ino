#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
Adafruit_MotorShield AFMS = Adafruit_MotorShield();
Adafruit_DCMotor *motorLeft = AFMS.getMotor(4);
Adafruit_DCMotor *motorRight = AFMS.getMotor(1);

void setup() 
{
  AFMS.begin();
}
void loop() 
{
  motorLeft->setSpeed(255); // Speed with 100% DC
  motorRight->setSpeed(255); // Speed with 100% DC
  motorLeft->run(FORWARD); // turn it on going forward
  motorRight->run(FORWARD); // turn it on going forward
  delay(2000);
}
