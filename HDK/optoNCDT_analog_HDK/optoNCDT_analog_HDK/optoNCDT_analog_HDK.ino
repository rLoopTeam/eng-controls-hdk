
int sensorPin1 = A0;
int sensorPin2 = A1;
int sensorPin3 = A2;

int sensorValue1 = 0;
int sensorValue2 = 0;
int sensorValue3 = 0;

void setup() {
  Serial.begin(9600);
}

void loop() {
  sensorValue1 = analogRead(sensorPin1);
  Serial.print("Sensor1:");
  Serial.println(sensorValue1);
  //Serial.print("Sensor2:");
  //Serial.println(sensorValue2);
  //Serial.print("Sensor3:");
  //Serial.println(sensorValue3);
  delay(100);

}
