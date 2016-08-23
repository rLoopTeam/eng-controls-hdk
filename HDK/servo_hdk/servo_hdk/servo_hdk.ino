#include <Servo.h>

#define step4 3
#define step3 4
#define step2 5
#define step1 6

Servo myservo1;
Servo myservo2;
Servo myservo3;
Servo myservo4;

int pos1 = 0;
int pos2 = 0;
int pos3 = 0;
int pos4 = 0;

void setup() 
{
  myservo1.attach(step1);
  myservo2.attach(step2);
  myservo3.attach(step3);
  myservo4.attach(step4);
  myservo2.write(0);
  myservo3.write(180);
  delay(1000);
}

void loop() 
{
  myservo1.write(55);
  myservo2.write(60);
  myservo3.write(60);
  myservo4.write(70);
  delay(500);
  myservo1.write(55);
  myservo2.write(60);
  myservo3.write(60);
  myservo4.write(70);
  delay(500);
}
