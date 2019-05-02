/**
 * IMU sensor
 * PolitTOcean year 2019
 */
 
#ifndef POLITOCEAN_IMU_H
#define POLITOCEAN_IMU_H

#include "Arduino.h"

class IMU {
  public:
    float pitch;
    float roll;
    void configure(); 
    void printValues();
    void imuRead();
  private:
    float Ax,Ay,Az,Gx,Gy,Gz,Tmp;
    float gyrData
    
};

#endif
