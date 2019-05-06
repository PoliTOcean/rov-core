#include <Wire.h>
#include "Arduino.h"
#include "PressureSensor.h"
#include "MS5837.h"

MS5837 sensor;

void PressureSensor::configure(){
  // initialize the MS5837 sensor
  sensor.init();

  //TODO check those flags utility
  reqPress = 0;
  savePressure = 0;
}

void PressureSensor::saveRequestedPressure(){
  reqPress = readPressure();
  savePressure = 0;
}
