#include <Servo.h>
#include <Wire.h>
#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#define step4 3
#define step3 4
#define step2 5
#define step1 6

#define pwmPin1 20
#define pwmPin2 21
#define pwmPin3 22
#define pwmPin4 23

Servo myservo1;
Servo myservo2; 
Servo myservo3;
Servo myservo4;

Adafruit_MMA8451 mma = Adafruit_MMA8451();

int pos1 = 0;
int pos2 = 0;
int pos3 = 0;
int pos4 = 0;

static double last_error;
static double point = 0.2;
static double voltage;
double Err = 0;
double iErr = 0;
double dErr = 0;
double kp;
double ki;
double kd;

void setup_constants(double p, double i, double d)
{
   kp = p;
   ki = i;
   kd = d;
}
void start_motors()
{
  analogWrite(pwmPin1, 249);
  analogWrite(pwmPin2, 249);
  analogWrite(pwmPin3, 249);
  analogWrite(pwmPin4, 249);
}
void stop_motors()
{
  analogWrite(pwmPin1, 125);
  analogWrite(pwmPin2, 125);
  analogWrite(pwmPin3, 125);
  analogWrite(pwmPin4, 125);
}
void setup() {  
  pinMode(pwmPin1, OUTPUT);
  pinMode(pwmPin2, OUTPUT);
  pinMode(pwmPin3, OUTPUT);
  pinMode(pwmPin4, OUTPUT);
  analogWriteFrequency(pwmPin1, 490);
  analogWriteFrequency(pwmPin2, 490);
  analogWriteFrequency(pwmPin3, 490);
  analogWriteFrequency(pwmPin4, 490);
  stop_motors();
  delay(6000);
  start_motors();
  
  myservo1.attach(step1);
  myservo2.attach(step2);
  myservo3.attach(step3);
  myservo4.attach(step4);
  myservo2.write(90);
  myservo3.write(90);

  myservo1.write(90);
  myservo4.write(90);
  delay(1000);

  Serial.begin(9600);
  Serial.println("Adafruit MMA8451 test!");
  
  if (! mma.begin()) {
    Serial.println("Couldnt start");
    while (1);
  }
  Serial.println("MMA8451 found!");
  
  mma.setRange(MMA8451_RANGE_2_G);
  
  Serial.print("Range = "); Serial.print(2 << mma.getRange());  
  Serial.println("G");
  delay(10);
  setup_constants(10, 0, 0);

}
void loop() {
/* TTTTTTTTEEEEEEEEEEESSSSSSSSSSSSSTTTTTTTTTTTT
  for (pos1 = 1; pos1 < 90; pos1++)
  {
    myservo2.write(pos1);
    myservo3.write(140-pos1);
    delay(5);  
  }
  for (pos1 = 90; pos1 >= 1; pos1--)
  {
    myservo2.write(pos1);
    myservo3.write(140-pos1);
    delay(5);
  }
*/  

  mma.read();
  sensors_event_t event;
  mma.getEvent(&event);
  Serial.print("X: \t"); Serial.println(event.acceleration.x);
  
  error_last = Err;
  input = event.acceleration.x;
  Err = point - input;

  if (Err < 0)
  {
    Err = -1 * Err;
  }
  
  dErr = Err - error_last;
  if (dErr < 0)
  {
    dErr = -dErr;
  }
  
  iErr = error_last + Err;
  if (iErr < 0)
  {
    iErr = -iErr;
  }
  voltage = kp * Err + ki * iErr + kd * dErr;
  
  
}
