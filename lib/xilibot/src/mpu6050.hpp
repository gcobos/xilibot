#ifndef mpu6050_h
#define mpu6050_h

#include <Wire.h>

void MPU6050_setup();

// Calibrate function. Take 100 readings (over 2 seconds) to calculate the gyro offset value. IMU should be steady in this process...
void MPU6050_calibrate();

// return true on new data available
bool MPU6050_newData();

void MPU6050_read_3axis();

// This function implements a complementary filter to fusion gyro and accel info
float MPU6050_getAngle(float dt);

#endif


