/**
 * IMU sensor
 * PolitTOcean year 2019
 */
 
#ifndef POLITOCEAN_IMU_H
#define POLITOCEAN_IMU_H

#include "Arduino.h"

class IMU {
  public:
    float getPitch();
    float getRoll();
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
