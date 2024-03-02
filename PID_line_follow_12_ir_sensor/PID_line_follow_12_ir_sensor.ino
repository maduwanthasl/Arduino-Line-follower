//Define motor pins
#define leftmotor_pwm 9
#define rightmotor_pwm 8
#define leftForward 26
#define rightForward 22
#define leftBackward 28
#define rightBackward 24

//define ir array sensors and calibration arrays and pins
const int numSensors = 12; // Number of IR sensors
const int sensorPins[numSensors] = {A7,A6,A15, A14, A13, A12, A11, A10, A9, A8,A5,A4}; // Sensor pin numbers
int sensorValues[numSensors]; // Array to store sensor values
int minValues[numSensors]; // Array to store minimum sensor values
int maxValues[numSensors]; // Array to store maximum sensor values
int threshold[numSensors]; // Array to store sensor thresholds   


//Initial Speed of Motor
int initial_motor_speed = 40;

// PID Constants
float Kp = 4;//18
float Ki = 1;//6
float Kd = 0;//8

float error = 0, P = 0, I = 0, D = 0, PID_value = 0;
float previous_error = 0, previous_I = 0;

int flag = 0;

void setup()
{

  pinMode(leftForward, OUTPUT);
  pinMode(leftBackward, OUTPUT);
  pinMode(rightForward, OUTPUT);
  pinMode(rightBackward, OUTPUT);
  pinMode(leftmotor_pwm, OUTPUT); // Changed from ENA
  pinMode(rightmotor_pwm, OUTPUT); // Changed from ENB

  Serial.begin(9600);                     //setting serial monitor at a default baund rate of 9600
  delay(5000);

  calibrateSensors(); // Calibrate the sensors
  delay(5000);

  Serial.println("Started !!");
  delay(1000);
}


/*-----------------------------------------------------------------------------------------------------------
----------------------------------------------SENSOR CALIBRATION---------------------------------------------
-----------------------------------------------------------------------------------------------------------*/

void calibrateSensors() {
  Serial.println("Calibrating sensors...");
  
  // Initialize minValues and maxValues arrays
  for (int i = 0; i < numSensors; i++) {
    minValues[i] = 1023; // Initialize with maximum possible value
    maxValues[i] = 0; // Initialize with minimum possible value
  }

  // Move motors to calibrate sensors
  for (int i = 0; i < 12000; i++) { // Adjust the loop count and motor speeds as needed
    analogWrite(leftmotor_pwm,45);
    analogWrite(rightmotor_pwm,45);
    digitalWrite(leftForward,HIGH);
    digitalWrite(rightBackward,HIGH);
    
    

    for (int sensorIndex = 0; sensorIndex < numSensors; sensorIndex++) {
      int sensorValue = analogRead(sensorPins[sensorIndex]);
      minValues[sensorIndex] = min(minValues[sensorIndex], sensorValue);
      maxValues[sensorIndex] = max(maxValues[sensorIndex], sensorValue);
    }
  }

  // Stop motors
  analogWrite(leftmotor_pwm*200/78,0);
  analogWrite(rightmotor_pwm,0);
  stop_bot();


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

/*------------------------------------------------------------------------------------------------------------
--------------------------------------------------------------------------------------------------------------
------------------------------------------------------------------------------------------------------------*/


void loop()
{
  read_sensor_values();

  if (error == 100) {               // Make left turn untill it detects straight path
    do {
      read_sensor_values();
      analogWrite(leftmotor_pwm, 70); //Changed from ENA
      analogWrite(rightmotor_pwm, 70); //Changed from ENB
      sharpLeftTurn();
      // Serial.println("error == 100");
    } while (error != 0 && error != -1 && error != 1);

  } 
  else if (error == 101) {          // Make right turn in case of it detects only right path (it will go into forward direction in case of staright and right "|--")
                                      // untill it detects straight path.
    analogWrite(leftmotor_pwm, 70); //Changed from ENA
    analogWrite(rightmotor_pwm, 70); //Changed from ENB
    forward();
    delay(150);
    stop_bot();
    read_sensor_values();
    if (error == 102) {
      do {
        analogWrite(leftmotor_pwm, 70); //Changed from ENA
        analogWrite(rightmotor_pwm, 70); //Changed from ENB
        sharpRightTurn();
        read_sensor_values();
      } while (error != 0 && error != -1 && error != 1);
    }
  } 
  else if (error == 102) {        // Make left turn untill it detects straight path
    do {
      analogWrite(leftmotor_pwm, 70); //Changed from ENA
      analogWrite(rightmotor_pwm, 70); //Changed from ENB
      sharpLeftTurn();
      read_sensor_values();
      if (error == 0 || error == 1 || error == -1) {
        stop_bot();
        delay(200);
      }
    }  while (error != 0 && error != -1 && error != 1);
  } 
  else if (error == 103) { // Make left turn untill it detects straight path or stop if dead end reached.
    if (flag == 0) {
      analogWrite(leftmotor_pwm, 70); //Changed from ENA
      analogWrite(rightmotor_pwm, 70); //Changed from ENB
      forward();
      delay(200);
      stop_bot();
      read_sensor_values();
      if (error == 103) {     /**** Dead End Reached, Stop! ****/
        stop_bot();
        flag = 1;
      } else {        /**** Move Left ****/
        analogWrite(leftmotor_pwm, 70); //Changed from ENA
        analogWrite(rightmotor_pwm, 70); //Changed from ENB
        sharpLeftTurn();
        delay(200);
        do {
          read_sensor_values();
          analogWrite(leftmotor_pwm, 70); //Changed from ENA
          analogWrite(rightmotor_pwm, 70); //Changed from ENB
          sharpLeftTurn();
        } while (error != 0 && error != -1 && error != 1);
      }
    }
  } 
  else {
    calculate_pid();
    motor_control();
  }
}


void read_sensor_values()
{
  //White == 1 & Black == 0 left - , right +
  for (int sensorIndex = 0; sensorIndex < numSensors; sensorIndex++) {
    sensorValues[sensorIndex] = analogRead(sensorPins[sensorIndex]);
    if (sensorValues[sensorIndex] > threshold[sensorIndex]) {
        sensorValues[sensorIndex] = 0;
    } else {
        sensorValues[sensorIndex] = 1;
    }
  }

  //print the values or the sensorVales array
  for (int i = 0; i < numSensors; i++) {
    Serial.print(sensorValues[i]);
    Serial.print(" ");
  }
  

  // Define all expected values in a single array
  int expectedValues[][numSensors] = {
    {0, 0, 0, 0, -1, -1, -1, -1, 0, 0, 0, 0},
    {0, 0, 0, 1, -1, -1, -1, -1, 1, 0, 0, 0},
    {0, 0, 0, 1, -1, -1, -1, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 1, -1, -1, -1, 1, 0, 0, 0},
    {0, 0, 0, 1, -1, -1, 1, 0, 0, 0, 0, 0},
    {0, 0, 1, -1, -1, -1, -1, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, -1, -1, 1, 0, 0, 0},
    {0, 0, 0, 0, 1, -1, -1, -1, -1, 1, 0, 0},
    {0, 0, 1, -1, -1, -1, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 1, -1, -1, -1, 1, 0, 0},
    {0 ,0 ,0 ,0 ,0 ,0 ,1 ,-1 ,-1 ,-1 ,1 ,0 },
    {0 ,1 ,-1 ,-1 ,-1 ,1 ,0 ,0 ,0 ,0 ,0 ,0 },
    {0, 0, 1, -1, -1, 1, 0, 0, 0, 0, 0, 0},
    {0, 1, -1, -1, -1, -1, 1, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 1, -1, -1, 1, 0, 0},
    {0, 0, 0, 0, 0, 1, -1, -1, -1, -1, 1, 0},
    {0, 1, -1, -1, 1, 0, 0, 0, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, -1, -1, 1, 0},
    {0, 0, 0, 0, 0, 0, 0, 0, 1, -1, 1, 0},
    {0, 1, -1, 1, 0, 0, 0, 0, 0, 0, 0, 0},    
    {1, -1, -1, -1, 1, 0, 0, 0, 0, 0, 0, 0},
    {1, -1, -1, -1, -1, 1, 0, 0, 0, 0, 0, 0},
    {1, -1, -1, -1, -1, -1, 1, 0, 0, 0, 0, 0},
    {1, -1, -1, -1, -1, -1, -1, 1, 0, 0, 0, 0},
    {0, 0, 0, 0, 0, 0, 0, 1, -1, -1, -1, 1},
    {0, 0, 0, 0, 0, 0, 1, -1, -1, -1, -1, 1},
    {0, 0, 0, 0, 0, 1, -1, -1, -1, -1, -1, 1},
    {0, 0, 0, 0, 1, -1, -1, -1, -1, -1, -1, 1},
    {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0},
    {-1, -1, 1, 1, 1, 1, 1, 1, 1, 1, -1, -1}
  };


  // Define corresponding error values
  int errorValues[] = {0, 0, -1, 1, -2, -2, 2, 2, -3, +3,4,-4, -4, -4,4,4, -5,5,6,-6,100,100,100,100,101,101,101,101,102,103};

  // Initialize conditionMet as false
  bool conditionMet = false;

  int defaultErrorValue = 0;
  

  // Check if any condition is met
  for (int condIndex = 0; condIndex < sizeof(expectedValues) / sizeof(expectedValues[0]); condIndex++) {
    conditionMet = true; // Assume the condition is met initially
    for (int sensorIndex = 0; sensorIndex < numSensors; sensorIndex++) {
      if (expectedValues[condIndex][sensorIndex] != -1 && sensorValues[sensorIndex] != expectedValues[condIndex][sensorIndex]) {
        conditionMet = false; // Condition not met, break the inner loop
        break;
      }
    }
    if (conditionMet) {
      error = errorValues[condIndex]; // Set error based on the condition
      break; // Exit the loop if condition is met
    }
  }

  // If no condition is met, set error to default value
  if (!conditionMet) {
    error = defaultErrorValue;
  }

  Serial.print("    ");
  Serial.print(error);
  Serial.println();
  delay(1000);
}


void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
  previous_error = error;
}

void motor_control()
{
  // Calculating the effective motor speed:
  int left_motor_speed = initial_motor_speed + PID_value;
  int right_motor_speed = initial_motor_speed - PID_value;

  // The motor speed should not exceed the max PWM value
  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  Serial.print(PID_value);
  Serial.print("\t");
  
  Serial.print(left_motor_speed);
  Serial.print("\t");
  Serial.println(right_motor_speed);
  delay(1000);


  

  analogWrite(leftmotor_pwm, left_motor_speed); //Left Motor Speed
  analogWrite(rightmotor_pwm, right_motor_speed); //Right Motor Speed

  //following lines of code are to make the bot move forward
  forward();
}


void forward()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
}

void reverse()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, HIGH);
}

void left()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
}

void right()
{
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(leftForward, HIGH);
  digitalWrite(leftBackward, LOW);
  digitalWrite(rightForward, LOW);
  digitalWrite(rightBackward, LOW);
}

void sharpLeftTurn() {
  /*The pin numbers and high, low values might be different depending on your connections */
  digitalWrite(leftForward, LOW);
  digitalWrite(leftBackward, HIGH);
  digitalWrite(rightForward, HIGH);
  digitalWrite(rightBackward, LOW);
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