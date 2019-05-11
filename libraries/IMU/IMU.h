/**
 * IMU sensor
 * PolitTOcean year 2019
 */
 
#ifndef POLITOCEAN_IMU_H
#define POLITOCEAN_IMU_H

#include "Arduino.h"


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

#define IMU_dT        0.03

class IMU {
  public:
    void configure(); 
    void printValues();
    void imuRead();
    void complementaryFilter();
    float pitch;
    float roll;
    
  private:
    float Ax,Ay,Az,Gx,Gy,Gz,Tmp;
    float gyrData;
    int dt;
    unsigned long lastUpdate;
    bool updatedValues;
};

#endif
