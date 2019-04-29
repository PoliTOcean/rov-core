#include "Arduino.h"
#include "FakeSensor.h"

  void FakeSensor::configure(){
    //iteration = 0;
    value = 5;
  }
  
  byte FakeSensor::getValue(){
      calculateValue();
      return value;
  }

  void FakeSensor::calculateValue(){
    value++;
  }
