#include <Time.h>
#include <avr/io.h>
#include <avr/interrupt.h>

#define PinInt1 14
#define sensorPin 33

volatile int t = 0;
volatile double time1 = 0;
volatile double time2 = 0;
volatile char flag = 0;
double length = 2;
volatile double velocity = 0;
void setup()
{
Serial.begin(115200);
pinMode(PinInt1, INPUT); // sets the digital pin as input
attachInterrupt(PinInt1, isrService1, RISING); // interrrupt 1 is data ready
pinMode(sensorPin, OUTPUT);
}

void isrService1()
{
cli();
time1 = micros();
flag = 1;
detachInterrupt(PinInt1);
attachInterrupt(PinInt1, isrService2, FALLING);
sei();
}

void isrService2()
{
cli();
time2 = micros();
flag = 2;
detachInterrupt(PinInt1);
attachInterrupt(PinInt1, isrService1, RISING);
sei();
}

void loop()
{ 
 if (flag != 0)
 {
   double q = 0;
   q = (time2 - time1) / 1000;
   if (q>0)
   {
   velocity = length / q;
   Serial.println(velocity);
   }
 }
 flag = 0;
}
