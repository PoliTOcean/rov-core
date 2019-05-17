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

#define DEF_IN_AXES_MIN 1
#define DEF_IN_AXES_MAX 254

#define MAX_IMU     80

#define SLOW_POWER    30
#define MEDIUM_POWER  60
#define FAST_POWER    100

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

    const float powerPerc[3] = {
      SLOW_POWER, MEDIUM_POWER, FAST_POWER
    };

    volatile int x, y, rz;
    
    bool savePressure;
    volatile int requested_pressure;
    
    volatile float input_axis_min, input_axis_max, motors_min, motors_max;

    Motor FL, FR, BL, BR, UR, UL, UB;

    PIDController pitchCorrection, rollCorrection, depthCorrection;
    

  public:
    enum power {
      SLOW, MEDIUM, FAST
    };

    volatile bool started;
    volatile float up;
    volatile int down;
    volatile Motors::power powerMode;
    bool configured = false;
    

    Motors( float dt,
            float in_axis_min   = DEF_IN_AXES_MIN,
            float in_axis_max   = DEF_IN_AXES_MAX,
            float motors_min    = DEF_MOTORS_MIN,
            float motors_max    = DEF_MOTORS_MAX) 
    :  input_axis_min(in_axis_min),
       input_axis_max(in_axis_max),
       motors_min(motors_min),
       motors_max(motors_max),
       FL(motors_min, motors_max, powerPerc[(int)power::SLOW]),
       FR(motors_min, motors_max, powerPerc[(int)power::SLOW]),       
       BL(motors_min, motors_max, powerPerc[(int)power::SLOW]),       
       BR(motors_min, motors_max, powerPerc[(int)power::SLOW]),       
       UL(motors_min, motors_max, powerPerc[(int)power::SLOW]),
       UR(motors_min, motors_max, powerPerc[(int)power::SLOW]),       
       UB(motors_min, motors_max, powerPerc[(int)power::SLOW]),               
       pitchCorrection(KP_pitch, KI_pitch, KD_pitch, dt, THRESHOLD_pitch, MAX_IMU),
       rollCorrection(KP_roll, KI_roll, KD_roll, dt, THRESHOLD_roll, MAX_IMU),
       depthCorrection(KP_depth, KI_depth, KD_depth, dt, THRESHOLD_depth, input_axis_max)
    {}

    void configure();

    void start(int current_pressure);
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
    
    void evaluateVertical(int current_pressure, float roll, float pitch);
    void evaluateHorizontal();
};


#endif
