#include <Wire.h>
#include <I2Cdev.h>
#include <MPU6050.h>

MPU6050 gyroAccelTemp;

float gyroDegree;
#define GYRO_Z_OFFSET 16
float gyroCalibrationFactor = 1.12; // Adjust this calibration factor empirically

bool isFirstLoopComplete;
float previousTime;

void gyro_setup() {
  Serial.begin(115200);
  Wire.begin();
  gyroAccelTemp.initialize();
  Serial.println("Testing device connections...");
  Serial.println(gyroAccelTemp.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  gyroAccelTemp.CalibrateGyro();
  Serial.println("Calibration complete");
  gyroAccelTemp.setZGyroOffset(GYRO_Z_OFFSET);
  gyroAccelTemp.CalibrateGyro(6);
  Serial.println("Fine Tuning Complete");
  isFirstLoopComplete = false;
}

void gyro_loop() {
  float gyroZ = gyroAccelTemp.getRotationZ() / 131.0 * gyroCalibrationFactor; // Apply calibration factor

  if (isFirstLoopComplete) {
    float timeForOneLoop = micros() - previousTime;
    gyroDegree += gyroZ * timeForOneLoop / 1000000.0;
  }
  previousTime = micros();
  
  if (!isFirstLoopComplete) {
    isFirstLoopComplete = true;
  }

  Serial.println(gyroDegree);
}
