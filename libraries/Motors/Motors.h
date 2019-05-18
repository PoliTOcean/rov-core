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
#include "PIDController.h"

#define DEF_AXIS_MIN -126
#define DEF_AXIS_MAX 127

#define MAX_IMU     80

#define H_SLOW_POWER    15
#define H_MEDIUM_POWER  50
#define H_FAST_POWER    100

#define V_SLOW_POWER    30
#define V_MEDIUM_POWER  60
#define V_FAST_POWER    100

#define KP_roll   250
#define KI_roll   0
#define KD_roll   0
#define THRESHOLD_roll  0.05    // 3 degrees

#define KP_pitch  250
#define KI_pitch  0
#define KD_pitch  0
#define THRESHOLD_pitch 0.05    // 3 degrees

#define KP_depth  50
#define KI_depth  0
#define KD_depth  0
#define THRESHOLD_depth 0.1     // 1 cm


class Motors {
  protected:
    const int signFL = -1;
    const int signFR = -1;
    const int signBL = 1;
    const int signBR = 1;
    const int signUR = 1;
    const int signUL = 1;
    const int signUB = 1;

    const float horizontalPowerPerc[3] = {
      H_SLOW_POWER, H_MEDIUM_POWER, H_FAST_POWER
    };

    const float verticalPowerPerc[3] = {
      V_SLOW_POWER, V_MEDIUM_POWER, V_FAST_POWER
    };

    volatile int x, y, rz;
    
    bool savePressure;
    volatile int requested_pressure;
    
    volatile float axis_min, axis_max;

    Motor FL, FR, BL, BR, UR, UL, UB;

    PIDController pitchCorrection, rollCorrection, depthCorrection;
    

  public:
    enum power {
      SLOW, MEDIUM, FAST
    };

    volatile bool started;
    volatile float up, down;
    volatile Motors::power powerMode;
    bool configured = false;
    

    Motors( float dt,
            float axis_min  = DEF_AXIS_MIN,
            float axis_max  = DEF_AXIS_MAX)
    :  axis_min(axis_min),
       axis_max(axis_max),
       FL(axis_min, axis_max, powerPerc[(int)power::SLOW]),
       FR(axis_min, axis_max, powerPerc[(int)power::SLOW]),
       BL(axis_min, axis_max, powerPerc[(int)power::SLOW]),
       BR(axis_min, axis_max, powerPerc[(int)power::SLOW]),
       UL(axis_min, axis_max, powerPerc[(int)power::SLOW]),
       UR(axis_min, axis_max, powerPerc[(int)power::SLOW]),
       UB(axis_min, axis_max, powerPerc[(int)power::SLOW]),
       pitchCorrection(KP_pitch, KI_pitch, KD_pitch, dt, THRESHOLD_pitch, MAX_IMU),
       rollCorrection(KP_roll, KI_roll, KD_roll, dt, THRESHOLD_roll, MAX_IMU),
       depthCorrection(KP_depth, KI_depth, KD_depth, dt, THRESHOLD_depth, axis_max)
    {}

    void configure();

    void start(int current_pressure);
    void stop();

    void setX(int x);
    void setY(int y);
    void setRz(int rz);
    
    void stopUp();
    void stopDown();
    void goUp();
    void goDown();
    void goUpFast();
    void stopUpFast();

    void setPower(Motors::power pwr);
    
    void evaluateVertical(int current_pressure, float roll, float pitch);
    void evaluateHorizontal();
};


#endif
