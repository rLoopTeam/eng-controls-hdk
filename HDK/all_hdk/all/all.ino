#include <Servo.h>
#include <Wire.h>
//#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

#define step4 3
#define step3 5
#define step2 4
#define step1 6

#define pwmPin1 20
#define pwmPin2 21
#define pwmPin3 22
#define pwmPin4 23

Servo myservo1;
Servo myservo2; 
Servo myservo3;
Servo myservo4;

//Adafruit_MMA8451 mma = Adafruit_MMA8451();
#define BNO055_SAMPLERATE_DELAY_MS (20)

Adafruit_BNO055 bno = Adafruit_BNO055(55);

int pos1 = 0;
int pos2 = 0;
int pos3 = 0;
int pos4 = 0;

static double input;
static double last_error;
static double point = 0;
static double voltage;
double Err = 0;
double iErr = 0;
double dErr = 0;
double kp;
double ki;
double kd;
bool direction;
unsigned long lastTime = 0;

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

void displaySensorDetails(void)
{
  sensor_t sensor;
  bno.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" xxx");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" xxx");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" xxx");
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displaySensorStatus(void)
{
  /* Get the system status values (mostly for debugging purposes) */
  uint8_t system_status, self_test_results, system_error;
  system_status = self_test_results = system_error = 0;
  bno.getSystemStatus(&system_status, &self_test_results, &system_error);

  /* Display the results in the Serial Monitor */
  Serial.println("");
  Serial.print("System Status: 0x");
  Serial.println(system_status, HEX);
  Serial.print("Self Test:     0x");
  Serial.println(self_test_results, HEX);
  Serial.print("System Error:  0x");
  Serial.println(system_error, HEX);
  Serial.println("");
  delay(500);
}

void displayCalStatus(void)
{
  /* Get the four calibration values (0..3) */
  /* Any sensor data reporting 0 should be ignored, */
  /* 3 means 'fully calibrated" */
  uint8_t system, gyro, accel, mag;
  system = gyro = accel = mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);

  /* The data should be ignored until the system calibration is > 0 */
  Serial.print("\t");
  if (!system)
  {
    Serial.print("! ");
  }

  /* Display the individual values */
  Serial.print("Sys:");
  Serial.print(system, DEC);
  Serial.print(" G:");
  Serial.print(gyro, DEC);
  Serial.print(" A:");
  Serial.print(accel, DEC);
  Serial.print(" M:");
  Serial.print(mag, DEC);
}

void set_servo(double myservo1_val, double myservo2_val, double myservo3_val, double myservo4_val)
{
  myservo1.write(myservo1_val - 0); // 90 - 45 == motor pointing downwards (when downwards is 90 degrees)
  myservo2.write(myservo2_val + 5);
  myservo3.write(myservo3_val - 5);
  myservo4.write(myservo4_val + 5);
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
  set_servo(90, 70, 90, 100);
  //set_servo(90, 90, 90, 90);
  delay(1000);

  Serial.begin(9600);
  Serial.println("Orientation Sensor Test"); Serial.println("");

  /* Initialise the sensor */
  if(!bno.begin())
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    while(1);
  }

  delay(1000);

  /* Display some basic information on this sensor */
  displaySensorDetails();

  /* Optional: Display current status */
  displaySensorStatus();

  bno.setExtCrystalUse(true);
  delay(10);
  //setup_constants(0.2, 0, 0);
  setup_constants(0.2, 0, 0);

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
  
  sensors_event_t event;
  bno.getEvent(&event);
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);
  last_error = Err;
  input = event.orientation.x;
  Err = point - input;

  if (input > 0 && input < 182){
    Err = -1*Err;
    direction = 0;
  } else if ( input > 182) {
    Err = Err + 360;
    //Err = Err + 360;
    direction = 1;
  }

//  if (Err < 0)
//  {
//    //Err = -1 * Err;
//    direction = 1;
//  }
//  else direction = 0;
    
  dErr = (Err - last_error)/timeChange;
  if (dErr < 0)
  {
    dErr = -dErr;
  }
  
  iErr = iErr + Err*timeChange;
  if (iErr < 0)
  {
    iErr = -iErr;
  }
  voltage = kp * Err + ki * iErr + kd * dErr;
  if (voltage > 20)
    voltage = 20;
  if (direction)
  {
    //myservo2.write(90);
    //myservo3.write(90);
    //myservo1.write(90 - voltage);
    //myservo4.write(90 + voltage);
    //set_servo(90 + voltage, 90, 90, 100 - voltage);
    //set_servo(90 + voltage, 90 + voltage, 90 - voltage, 100 - voltage);
    set_servo(90, 90 - voltage, 90, 90 + voltage);
  }
  else if (!direction)
  {
    //myservo1.write(90);
    //myservo4.write(90);
    //myservo2.write(90 + voltage);
    //myservo3.write(90 - voltage);
    //set_servo(90, 95 - voltage, 90 + voltage, 90); 
    //set_servo(90 - voltage, 95 - voltage, 90 + voltage, 90 + voltage); 
    set_servo(90 + voltage, 90, 90 - voltage, 90);
  }
  lastTime = now;
    
    /* Display the floating point data */
  Serial.print("X: ");
  Serial.print(event.orientation.x, 4);
  Serial.print("\tY: ");
  Serial.print(event.orientation.y, 4);
  Serial.print("\tZ: ");
  Serial.print(event.orientation.z, 4);
  Serial.print("\tVoltage: ");
  Serial.print(voltage, 4);
  Serial.print("\tDir: ");
  Serial.print(direction, 4);
  Serial.print("\tErr: ");
  Serial.print(Err, 4);

  /* Optional: Display calibration status */
  displayCalStatus();

  /* Optional: Display sensor status (debug only) */
  //displaySensorStatus();

  /* New line for the next sample */
  Serial.println("");

  /* Wait the specified delay before requesting nex data */
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

