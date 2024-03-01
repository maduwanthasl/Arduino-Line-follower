#define leftmotor_pwm 9
#define rightmotor_pwm 8
#define leftForward 26
#define rightForward 22
#define leftBackward 28
#define rightBackward 24

const int speed = 71;

const int numSensors = 8; // Number of IR sensors
const int sensorPins[numSensors] = {A8, A9, A10, A11, A12, A13, A14, A15}; // Sensor pin numbers
int sensorValues[numSensors]; // Array to store sensor values
int minValues[numSensors]; // Array to store minimum sensor values
int maxValues[numSensors]; // Array to store maximum sensor values
int threshold[numSensors]; // Array to store sensor thresholds

void setup() {
  Serial.begin(9600);
  analogWrite(leftmotor_pwm, speed);
  analogWrite(rightmotor_pwm, speed);
  //calibrateSensors(); // Calibrate the sensors
}

void loop() {
  // Your main code goes here
  for (int sensorIndex = 0; sensorIndex < numSensors; sensorIndex++) {
      sensorValues[sensorIndex] = analogRead(sensorPins[sensorIndex]);
  }
  for (int i = 0; i < numSensors; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  Serial.println();
  delay(1000);

}

void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  
  // Initialize minValues and maxValues arrays
  for (int i = 0; i < numSensors; i++) {
    minValues[i] = 1023; // Initialize with maximum possible value
    maxValues[i] = 0; // Initialize with minimum possible value
  }

  // Move motors to calibrate sensors
  for (int i = 0; i < 6000; i++) { // Adjust the loop count and motor speeds as needed
    //sharpRightTurn();

    for (int sensorIndex = 0; sensorIndex < numSensors; sensorIndex++) {
      int sensorValue = analogRead(sensorPins[sensorIndex]);
      minValues[sensorIndex] = min(minValues[sensorIndex], sensorValue);
      maxValues[sensorIndex] = max(maxValues[sensorIndex], sensorValue);
    }
  }

  // Stop motors
  //stop_bot();


  // Calculate thresholds
  for (int i = 0; i < numSensors; i++) {
    threshold[i] = (minValues[i] + maxValues[i]) / 2;
  }

  // Print thresholds
  Serial.println("Sensor thresholds:");
  for (int i = 0; i < numSensors; i++) {
    Serial.print("Sensor ");
    Serial.print(i + 1);
    Serial.print(": ");
    Serial.println(threshold[i]);
  }

  Serial.println("Calibration complete.");
}

void sharpRightTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);
}

void stop_bot()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
}

