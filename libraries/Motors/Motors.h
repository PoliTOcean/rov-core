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
    volatile byte x,y,rz;
    volatile bool started;
    volatile int up,down;
    volatile int velocity;
    
    void configure(MS5837 psensor,IMU imu);

    void start();
    void stop();
    
    void stopVertical();
    void goUp();
    void goDown();
    
    void evaluateVertical();
    void evaluateHorizontal();
    
  protected:
    const int signFL = -1;
    const int signFR = 1;
    const int signBL = -1;
    const int signBR = 1;
    const int signUR = 1;
    const int signUL = 1;
    const int signUB = 1;

    Motor FL, FR, BL, BR, UR, UL, UB;
    
    MS5837 brSensor;
    bool savePressure;
    float reqPress;
  
    IMU imuSensor;
    float calcPitchPower();
    float calcRollPower();
};

#endif
