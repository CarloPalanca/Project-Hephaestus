#include <Servo.h>
#define lposition1  160
#define rposition1  55
#define lposition2  150
#define rposition2  150
Servo leftServo;
Servo rightServo;

void setup() 
{
  
  //leftServo.write(lposition1);
  rightServo.write(rposition1);
  delay(3000);
  rightServo.attach(9); 
  leftServo.attach(10);
  leftServo.write(lposition2);
  rightServo.write(rposition2);
}

void loop() {}
