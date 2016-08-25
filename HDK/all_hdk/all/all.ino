/*
 * HDK code
 * Uses PID controller to stabilise the HDK
 */
#include <Servo.h>
#include <Wire.h>
//#include <Adafruit_MMA8451.h>
#include <Adafruit_Sensor.h>

#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>

// servo pins
#define step4 3
#define step3 5
#define step2 4
#define step1 6

// hover engine pins
#define pwmPin1 20
#define pwmPin2 21
#define pwmPin3 22
#define pwmPin4 23

// servo initialisation
Servo myservo1;
Servo myservo2; 
Servo myservo3;
Servo myservo4;

//Adafruit_MMA8451 mma = Adafruit_MMA8451();
#define BNO055_SAMPLERATE_DELAY_MS (20)

// bno055 orientation sensor
Adafruit_BNO055 bno = Adafruit_BNO055(55);

int pos1 = 0;
int pos2 = 0;
int pos3 = 0;
int pos4 = 0;

// variables for PID controller
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

/*
 * Function for setting up the three PID constants
 */
void setup_constants(double p, double i, double d)
{
   kp = p;
   ki = i;
   kd = d;
}

/*
 * Function for starting the motors (hover engines)
 */
void start_motors()
{
  analogWrite(pwmPin1, 249);
  analogWrite(pwmPin2, 249);
  analogWrite(pwmPin3, 249);
  analogWrite(pwmPin4, 249);
}
/*
 * Function for stopping the motors (hover engines)
 */
void stop_motors()
{
  analogWrite(pwmPin1, 125);
  analogWrite(pwmPin2, 125);
  analogWrite(pwmPin3, 125);
  analogWrite(pwmPin4, 125);
}

/*
 * Function for displaying the sensors details at initialisation
 */
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

/*
 * Function for displaying the status of the sensors
 */
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

/*
 * Function for setting the servos including a calibration value
 * Parameters match the hover engine/servo number on the HDK 
 * e.g. 1 - servo 1, 2 - servo 2 etc
 */
void set_servo(double myservo1_val, double myservo2_val, double myservo3_val, double myservo4_val)
{
  // when setting the value of a servo, we include a correction value to account for hardware issues.
  // the correction vlaue is simply the correction angle required to make the gimbal point straight down.
  // if we set the servo to point to 90 degrees(striahgt down) but the gimbal points 5 degrees off, we have to add to remove
  // 5 degrees in the write() function parameter as shown below.
  myservo1.write(myservo1_val - 0); // 90 - 45 == motor pointing downwards (when downwards is 90 degrees)
  myservo2.write(myservo2_val + 5);
  myservo3.write(myservo3_val - 5);
  myservo4.write(myservo4_val + 5);
}

/*
 * Main setup function. 
 * Reset the HDK, initialise sensors and PID and start motors
 */
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


/*
 * Main loop function.
 * Run PID to adjust servos, to stabilise HDK
 */
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
  
  // update sensor
  bno.getEvent(&event);
  
  unsigned long now = millis();
  double timeChange = (double)(now - lastTime);
  
  // lagging variable for P error
  last_error = Err;
  
  // get orientation from sensor
  input = event.orientation.x;
  
  // calculate error from intended orientation (currently set to an absolute value of 0 (like a compass pointing north), but could be adjusted, or even made relative)
  Err = point - input;

  // if the sensor is returning a value between 0 and 180, set reverse the error value and set directio to 0
  if (input > 0 && input < 180){
    Err = -1*Err;
    direction = 0;
  } else if ( input > 180) { // if sensor is on other side, modify the error
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
  
  // differnetial error calculation
  dErr = (Err - last_error)/timeChange;
  if (dErr < 0)
  {
    dErr = -dErr;
  }
  
  // integral error calculation
  iErr = iErr + Err*timeChange;
  if (iErr < 0)
  {
    iErr = -iErr;
  }
  
  // final PID value
  voltage = kp * Err + ki * iErr + kd * dErr;
  
  // cap the voltage value to 20 because any more and the gimbal will hit the ground or the top plate
  if (voltage > 20)
    voltage = 20;
  
  // set servo values according to the direction and apply the PID value
  if (direction)
  {
    //myservo2.write(90);
    //myservo3.write(90);
    //myservo1.write(90 - voltage);
    //myservo4.write(90 + voltage);
    //set_servo(90 + voltage, 90, 90, 100 - voltage);
    //set_servo(90 + voltage, 90 + voltage, 90 - voltage, 100 - voltage);
    set_servo(90, 90 - voltage, 90, 90 + voltage); // set two diagonal motors to always point downwards, and the other two to point according to the PID
  }
  else if (!direction)
  {
    //myservo1.write(90);
    //myservo4.write(90);
    //myservo2.write(90 + voltage);
    //myservo3.write(90 - voltage);
    //set_servo(90, 95 - voltage, 90 + voltage, 90); 
    //set_servo(90 - voltage, 95 - voltage, 90 + voltage, 90 + voltage); 
    set_servo(90 + voltage, 90, 90 - voltage, 90); // set two diagonal motors to always point downwards, and the other two to point according to the PID
  }
  
  // used for time calculation
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

