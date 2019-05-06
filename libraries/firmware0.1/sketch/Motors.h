/**
 * Motors sensor
 * PolitTOcean year 2019
 */
 
#ifndef POLITOCEAN_MOTORS_H
#define POLITOCEAN_MOTORS_H

#include "Arduino.h"
#include "DumpedCurrentMotor.h"
#include "MS5837.h"
#include "IMU.h"

class Motors {
  public:
    void configure(MS5837 psensor,IMU imu);
    void control();
    volatile byte x,y,rz;
    volatile bool started;
    volatile int up,down;
    void start();
    void stop();
    void stopVertical();
    void goUp();
    void goDown();
    volatile int velocity;
    void evaluateVertical();
    void evaluateHorizontal();
    
  private:
    const int signFL = -1;
    const int signFR = 1;
    const int signBL = -1;
    const int signBR = 1;
    const int signUR = 1;
    const int signUL = 1;
    const int signUB = 1;

    bool savePressure;

    float reqPress;
  
    Motor FL, FR, BL, BR, UR, UL, UB;
    MS5837 brSensor;
    IMU imuSensor;
    float calcPitchPower();
    float calcRollPower();
};

#endif
