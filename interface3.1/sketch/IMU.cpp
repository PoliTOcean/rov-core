#include <Wire.h>
#include "Arduino.h"
#include "IMU.h"

#define MPU_ADDR      0x68
#define ACCEL_XOUT_H  0x3B
#define ACCEL_XOUT_L  0x3C
#define ACCEL_YOUT_H  0x3D
#define ACCEL_YOUT_L  0x3E
#define ACCEL_ZOUT_H  0x3F
#define ACCEL_ZOUT_L  0x40
#define TEMP_OUT_H    0x41
#define TEMP_OUT_L    0x42
#define GYRO_XOUT_H   0x43
#define GYRO_XOUT_L   0x44
#define GYRO_YOUT_H   0x45
#define GYRO_YOUT_L   0x46
#define GYRO_ZOUT_H   0x47
#define GYRO_ZOUT_L   0x48

void IMU::configure(){
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void IMU::updateValues(){
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // request a total of 14 registers
  Ax = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  Ay = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  Az = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  //Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  Gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  Gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  Gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
}

void IMU::printValues(){

  updateValues();
  
  Serial.print("Acc:\t");
  Serial.print("X= "); Serial.print(Ax);
  Serial.print("\tY= "); Serial.print(Ay);
  Serial.print("\tZ= "); Serial.println(Az);
  //equation for temperature in degrees C from datasheet
  //Serial.print("Temperature: "); Serial.print(Tmp / 340.00 + 36.53); Serial.println(" C ");

  Serial.print("Gyr:\t");
  Serial.print("X= "); Serial.print(Gx);
  Serial.print("\tY= "); Serial.print(Gy);
  Serial.print("\tZ= "); Serial.println(Gz);
  Serial.println("****");
}

void IMU::plotAcc(){

  updateValues();
  
  Serial.print(Ax);
  Serial.print(",");
  Serial.print(Ay);
  Serial.print(",");
  Serial.println(Az);
}


void IMU::plotGyro(){

  updateValues();
  
  Serial.print(Gx);
  Serial.print(",");
  Serial.print(Gy);
  Serial.print(",");
  Serial.println(Gz);
}

byte IMU::getAx(){
  updateValues();   
  return ((Ax/256) & 0x00FF);
}

byte IMU::getAy(){
  updateValues();
  return ((Ay/256) & 0x00FF);
}

byte IMU::getAz(){
  updateValues();
  return ((Az/256) & 0x00FF);
}


byte IMU::getGx(){
  updateValues();
  return ((Gx/256) & 0x00FF);
}

byte IMU::getGy(){
  updateValues();
  return ((Gy/256) & 0x00FF);  
}

byte IMU::getGz(){
  updateValues();
  return ((Gz/256) & 0x00FF);  
}
