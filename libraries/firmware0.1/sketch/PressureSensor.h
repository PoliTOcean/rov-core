/**
 * IMU sensor
 * PolitTOcean year 2019
 */
 
#ifndef POLITOCEAN_PRESSURESENSOR_H
#define POLITOCEAN_PRESSURESENSOR_H

#include "Arduino.h"

class PressureSensor {
  public:
    void configure();
    int readPressure();
    void saveRequestedPressure();
    int reqPress;
    int savePressure;
  private:
  
};

#endif
