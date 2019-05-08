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
    volatile bool started;
    volatile float up;
    volatile int down;
    volatile int powerMode;
        
    void configure(MS5837 psensor, IMU imu);

    void start();
    void stop();

    void setX(byte x);
    void setY(byte y);
    void setRz(byte rz);
    
    void stopUp();
    void stopDown();
    void goUp();
    void goDown();
    void goUpFast();
    void stopUpFast();

    void setPower(int power);
    
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

    volatile byte x,y,rz;
    
    MS5837 brSensor;
    bool savePressure;
    float reqPress;

    bool configured = false;

    IMU imuSensor;
    float calcPitchPower();
    float calcRollPower();
};

#endif
