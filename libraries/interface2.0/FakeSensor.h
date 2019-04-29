/**
 * Fake Sensor
 * PolitTOcean year 2019
 */
 
#ifndef POLITOCEAN_FAKESENSOR_H
#define POLITOCEAN_FAKESENSOR_H

#include "Arduino.h"

class FakeSensor
{
  public:
    void configure(); 
    byte getValue();
  private:
    volatile int iteration;
    volatile byte value;
    void calculateValue();
};

#endif
