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

#define SLOW_POWER    0.5
#define MEDIUM_POWER  1.3
#define FAST_POWER    3.3

class Motors {
  public:
    enum power {
      SLOW, MEDIUM, FAST
    };

    volatile bool started;
    volatile float up;
    volatile int down;
    volatile Motors::power powerMode;
        
    void configure(MS5837 *psensor, IMU imu);

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

    void setPower(Motors::power pwr);
    
    void evaluateVertical();
    void evaluateHorizontal();

    void setCurrentPressure(float currPress);
    
  protected:
    const int signFL = -1;
    const int signFR = -1;
    const int signBL = 1;
    const int signBR = 1;
    const int signUR = 1;
    const int signUL = 1;
    const int signUB = 1;

    const float mulPower[3] = {
      SLOW_POWER, MEDIUM_POWER, FAST_POWER
    };

    Motor FL, FR, BL, BR, UR, UL, UB;

    volatile int x,y,rz;
    
    MS5837 *brSensor;
    bool savePressure;
    float requested_pressure;
    float current_pressure;

    bool configured = false;

    IMU imuSensor;
    float calcPitchPower();
    float calcRollPower();
};


#endif
