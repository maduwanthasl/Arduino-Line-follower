
const int sensorPin1 = A15;
const int sensorPin2 = A14;
const int sensorPin3 = A13;
const int sensorPin4 = A12;
const int sensorPin5 = A11;
const int sensorPin6 = A10;
const int sensorPin7 = A9;
const int sensorPin8 = A8;

void setup() {
  Serial.begin(9600);
}

void loop() {
  int sensorValue1 = analogRead(sensorPin1);
  int sensorValue2 = analogRead(sensorPin2);
  int sensorValue3 = analogRead(sensorPin3);
  int sensorValue4 = analogRead(sensorPin4);
  int sensorValue5 = analogRead(sensorPin5);
  int sensorValue6 = analogRead(sensorPin6);
  int sensorValue7 = analogRead(sensorPin7);
  int sensorValue8 = analogRead(sensorPin8);
  
  // print out the value you read:
  Serial.print(sensorValue1);
  Serial.print("   ");
  Serial.print(sensorValue2);
  Serial.print("   ");
  Serial.print(sensorValue3);
  Serial.print("   ");
  Serial.print(sensorValue4);
  Serial.print("   ");
  Serial.print(sensorValue5);
  Serial.print("   ");
  Serial.print(sensorValue6);
  Serial.print("   ");
  Serial.print(sensorValue7);
  Serial.print("   ");
  Serial.println(sensorValue8);
  delay(1000);
}
