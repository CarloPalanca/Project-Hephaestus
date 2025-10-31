#include <AFMotor.h>

AF_DCMotor motor1(1, MOTOR12_1KHZ);
AF_DCMotor motor2(4, MOTOR34_1KHZ);

#define echoPin A5
#define trigPin A4
#define forward   255
#define reverse   255
#define turn      255
#define factor    0.8


int maximumRange = 15;
int minimumRange = 0;
long duration, distance;
int AINL = digitalRead(A1);
int AINC = digitalRead(A2);
int AINR = digitalRead(A3);

void setup()
{
  Serial.begin(9600);
  pinMode(A1, INPUT);
  pinMode(A2, INPUT);
  pinMode(A3, INPUT);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

}

void loop()
{
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
    motor1.setSpeed(reverse);
    motor1.run(BACKWARD);
    motor2.setSpeed(reverse);
    motor2.run(BACKWARD);
  }
  else if (AINL && AINC && !AINR) //110
  {
    Serial.println("110");
    motor1.setSpeed(reverse * factor);
    motor1.run(BACKWARD);
    motor2.setSpeed(reverse);
    motor2.run(RELEASE);
  }
  else if (!AINL && AINC && AINR) //011
  {
    Serial.println("011");
    motor1.setSpeed(reverse);
    motor1.run(RELEASE);
    motor2.setSpeed(reverse * factor);
    motor2.run(BACKWARD);
  }
  else if (AINL && !AINC && !AINR) //100
  {
    Serial.println("100");
    motor1.setSpeed(reverse * factor);
    motor1.run(BACKWARD);
    motor2.setSpeed(reverse);
    motor2.run(BACKWARD);
  }
  else if (!AINL && !AINC && AINR) //001
  {
    Serial.println("001");
    motor1.setSpeed(reverse);
    motor1.run(BACKWARD);
    motor2.setSpeed(reverse * factor);
    motor2.run(BACKWARD);
  }
  else if (AINL && AINC && AINR) //111
  {
    if (distance <= maximumRange)
    {
      Serial.println(distance);
      motor1.setSpeed(forward);
      motor1.run(FORWARD);
      motor2.setSpeed(forward);
      motor2.run(FORWARD);
    }
    else
    {
      Serial.println("-1");
      motor1.setSpeed(reverse * factor);
      motor1.run(BACKWARD);
      motor2.setSpeed(forward * factor);
      motor2.run(FORWARD);
      delay(500);
    }
  }
  delay(50);
}
