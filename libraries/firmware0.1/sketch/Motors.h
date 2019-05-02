/**
 * Motors sensor
 * PolitTOcean year 2019
 */
 
#ifndef POLITOCEAN_MOTORS_H
#define POLITOCEAN_MOTORS_H

#include "Arduino.h"
#include "DumpedCurrentMotor.h"
#include "PressureSensor.h"
#include "IMU.h"

class Motors {
  public:
    void configure(PressureSensor psensor,IMU imu);
    void control();
  private:
    Motor M0,M1,M2,M3,M4,M5,M6;
    PressureSensor pressureSensor;
    IMU imuSensor;
    void evaluateVertical(float kAng, float kDep, int vertical[4]);
    void evaluateHorizontal(int *leftFront,int  *rightFront,int  *leftBack,int  *rightBack);
    float calcPitchPower(float kAng);
    float calcRollPower(float kAng);
    
};

#endif
