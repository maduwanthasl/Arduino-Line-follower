#include <util/atomic.h> // For the ATOMIC_BLOCK macro

#define ENCA 2 // YELLOW
#define ENCB 3 // WHITE
#define PWM 9
#define IN2 26
#define IN1 28

int pos = 0; 
long prevT = 0;
float eprev = 0;
float eintegral = 0;            
float kp = 1;
float kd = 0.025;
float ki = 0.0;
int rpe = 2390; // encoders per round
int rpe_count = 1; // Forward 1 & reverse -1
int target_approx = rpe * rpe_count - 5;

void setup() {
  Serial.begin(9600);
  pinMode(ENCA, INPUT);
  pinMode(ENCB, INPUT);
  attachInterrupt(digitalPinToInterrupt(ENCA), readEncoder, RISING);
  
  pinMode(PWM, OUTPUT);
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
}

void loop() {
  if (pos >= target_approx && target_approx > 0) {
    // Stop the motor
    setMotor(0, 0, PWM, IN1, IN2);
    // Break the loop
    return;
    }
  if (pos < target_approx && target_approx < 0) {
    // Stop the motor
    setMotor(0, 0, PWM, IN1, IN2);
    // Break the loop
    return;
    }

  // time difference
  long currT = micros();
  float deltaT = ((float) (currT - prevT)) / (1.0e6);
  prevT = currT;

  // error
  int e = pos - rpe * rpe_count;

  // derivative
  float dedt = (e - eprev) / (deltaT);

  // integral
  eintegral = eintegral + e * deltaT;

  // control signal
  float u = kp * e + kd * dedt + ki * eintegral;

  // motor power
  float pwr = fabs(u);
  if (pwr > 75) {
    pwr = 75;
  } else if (pwr < 30) { // Add this condition to limit pwr above 30
    pwr = 30;
  }

  // motor direction
  int dir = 1;
  if(u<0){
    dir = -1;
  }

  // signal the motor
  setMotor(dir, pwr, PWM, IN1, IN2);

  // store previous error
  eprev = e;

}


void setMotor(int dir, int pwmVal, int pwm, int in1, int in2) {
  analogWrite(pwm, pwmVal);
  if (dir == 1) {
    digitalWrite(in1, HIGH);
    digitalWrite(in2, LOW);
  } else if (dir == -1) {
    digitalWrite(in1, LOW);
    digitalWrite(in2, HIGH);
  } else {
    digitalWrite(in1, LOW);
    digitalWrite(in2, LOW);
  }  
}

void readEncoder() {
  int b = digitalRead(ENCB);
  if (b > 0) {
    pos++;
  } else {
    pos--;
  }
}
