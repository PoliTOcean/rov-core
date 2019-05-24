/**
 * Motors sensor
 * PolitTOcean year 2019
 */
 
#ifndef POLITOCEAN_MOTORS_H
#define POLITOCEAN_MOTORS_H

#include "Arduino.h"
#include "DampedCurrentMotor.h"
#include "PressureSensor.h"
#include "IMU.h"
#include "PIDController.h"

#define DEF_AXIS_MIN -126
#define DEF_AXIS_MAX 127

#define OFFSET_POWER    40

#define TIME_TO_REACH_MAX 3 //seconds
#define DEF_TIME_TO_UPDATE_MS TIME_TO_REACH_MAX*H_POWER*10 // time*h_power_perc/100 * 1000
#define H_POWER         1
#define H_SLOW_POWER    20
#define H_MEDIUM_POWER  50
#define H_FAST_POWER    100

#define V_POWER         1
#define V_SLOW_POWER    25
#define V_MEDIUM_POWER  50
#define V_FAST_POWER    100

#define KP_roll   400
#define KI_roll   0
#define KD_roll   0
#define THRESHOLD_roll  0.03    // 1.4 degrees

#define KP_pitch  400
#define KI_pitch  0
#define KD_pitch  0
#define THRESHOLD_pitch 0.03    // 1.4 degrees

#define KP_depth  30
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
    
    long long time_to_update, last_update;

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
            float axis_max  = DEF_AXIS_MAX,
            int time_to_update_ms = DEF_TIME_TO_UPDATE_MS)
    :  axis_min(axis_min),
       axis_max(axis_max),
       FL(axis_min, axis_max, 0, 0, H_POWER),
       FR(axis_min, axis_max, 0, 0, H_POWER),
       BL(axis_min, axis_max, 0, 0, H_POWER),
       BR(axis_min, axis_max, 0, 0, H_POWER),
       UL(axis_min, axis_max, OFFSET_POWER, 0, V_POWER),
       UR(axis_min, axis_max, OFFSET_POWER, 0, V_POWER),
       UB(axis_min, axis_max, OFFSET_POWER, 0, V_POWER),
       pitchCorrection(KP_pitch, KI_pitch, KD_pitch, dt, THRESHOLD_pitch, axis_max),
       rollCorrection(KP_roll, KI_roll, KD_roll, dt, THRESHOLD_roll, axis_max),
       depthCorrection(KP_depth, KI_depth, KD_depth, dt, THRESHOLD_depth, axis_max),
       time_to_update((long)time_to_update_ms * 1000),
       last_update(0)
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

    void writeMotors();
};


#endif
