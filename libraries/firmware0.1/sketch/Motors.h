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
    byte x,y,rz;
    bool started;
    int up,down;
    void start(); // TODO implementnt
    void stop();  // TODO implementnt
    void stopVertical();  // TODO implementnt
    void goUp();  // TODO implementnt
    void goDown();  // TODO implementnt
    int velocity;
    
  private:
    Motor M0,M1,M2,M3,M4,M5,M6;
    MS5837 pressureSensor;
    IMU imuSensor;
    void evaluateVertical(float kAng, float kDep, int vertical[4]);
    void evaluateHorizontal(int *leftFront,int  *rightFront,int  *leftBack,int  *rightBack);
    float calcPitchPower(float kAng);
    float calcRollPower(float kAng);
};

#endif
