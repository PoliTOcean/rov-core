/**
 * IMU sensor
 * PolitTOcean year 2019
 */
 
#ifndef POLITOCEAN_IMU_H
#define POLITOCEAN_IMU_H

#include "Arduino.h"

class IMU {
  public:
    void configure(); 
    byte getAx();
    byte getAy();
    byte getAz();
    byte getGx();
    byte getGy();
    byte getGz();
    void printValues();
    void plotAcc();
    void plotGyro();
  private:
    int16_t Ax,Ay,Az,Gx,Gy,Gz;
    void updateValues();
    
};

#endif
