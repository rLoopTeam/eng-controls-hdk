
#define pwmPin1 20
#define pwmPin2 21
#define pwmPin3 22
#define pwmPin4 23
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
}
void loop() {
}
