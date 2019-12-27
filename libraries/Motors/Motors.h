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
#include "RBD_Timer.h"

#define DEF_AXIS_MIN -126
#define DEF_AXIS_MAX 127

//value definition for the up and down axis
#define VERTICAL_AXIS_MIN -32767
#define VERTICAL_AXIS_MAX 32766  
#define VERTICAL_AXIS_PROTECT 100

#define V_OFFSET_POWER      40
#define H_SLOW_OFFSET_POWER 20  //slow
#define H_MF_OFFSET_POWER   30  //medium and fast

#define TIME_TO_REACH_MAX 2.5 //seconds
#define DEF_TIME_TO_UPDATE_MS TIME_TO_REACH_MAX*H_POWER_STEP*10 // time*h_power_perc/100 * 1000
#define H_POWER_STEP    1
#define H_SLOW_POWER    30
#define H_MEDIUM_POWER  60
#define H_FAST_POWER    100

#define V_POWER_STEP    1
#define V_SLOW_POWER    40
#define V_MEDIUM_POWER  60
#define V_FAST_POWER    100

#define KP_roll   0
#define KI_roll   0
#define KD_roll   0

#define KP_pitch  0
#define KI_pitch  0
#define KD_pitch  0

#define KU_depth  40
#define PU_depth  3.31

#define KP_depth  KU_depth/3              //0.6*KU_depth        //KU_depth/5              
#define KI_depth  0.666*KU_depth/PU_depth //2*KP_depth/PU_depth //0.4*KU_depth/PU_depth   
#define KD_depth  KU_depth*PU_depth/9     //KP_depth*PU_depth/8 //KU_depth*PU_depth/15    


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

    //useless after the new feature
    // const float verticalPowerPerc[3] = {
    //   V_SLOW_POWER, V_MEDIUM_POWER, V_FAST_POWER
    // };

    volatile int x, y, rz;
    volatile int up,down;
    volatile bool savePressure;
    bool countingTimeForPressure;
    long long startTimeForPressure;
    float requested_pressure;
    volatile float axis_min, axis_max;
    Motor FL, FR, BL, BR, UR, UL, UB;
    PIDController pitchCorrection, rollCorrection, depthCorrection;
    RBD::Timer timer;
    int prev_rz;

    //int calcRzOffsetPower(int prev_rz, int rz);
    
  public:
    enum power {
      SLOW, MEDIUM, FAST
    };

    volatile bool started;
    //volatile float up, down;
    volatile Motors::power powerMode;
    bool configured = false;
    

    Motors( float dt,
            float axis_min  = DEF_AXIS_MIN,
            float axis_max  = DEF_AXIS_MAX,
            int time_to_update_ms = DEF_TIME_TO_UPDATE_MS)
    :  axis_min(axis_min),
       axis_max(axis_max),
       FL(axis_min, axis_max, 0, 0, H_POWER_STEP),
       FR(axis_min, axis_max, 0, 0, H_POWER_STEP),
       BL(axis_min, axis_max, 0, 0, H_POWER_STEP),
       BR(axis_min, axis_max, 0, 0, H_POWER_STEP),
       UL(axis_min, axis_max, V_OFFSET_POWER, 0, V_POWER_STEP),
       UR(axis_min, axis_max, V_OFFSET_POWER, 0, V_POWER_STEP),
       UB(axis_min, axis_max, V_OFFSET_POWER, 0, V_POWER_STEP),
       pitchCorrection(KP_pitch, KI_pitch, KD_pitch, dt, axis_max),
       rollCorrection(KP_roll, KI_roll, KD_roll, dt, axis_max),
       depthCorrection(KP_depth, KI_depth, KD_depth, dt, axis_max)
    {
      timer.setTimeout(time_to_update_ms);
    }

    void configure();

    void start();
    void stop();

    void setX(int x);
    void setY(int y);
    void setRz(int rz);
    
    void setUP(int up);
    void setDOWN(int down);
    // void stopUp();
    // void stopDown();
    // void goUp();
    // void goDown();
    // void goUpFast();
    // void stopUpFast();

    void setPower(Motors::power pwr);
    
    void evaluateVertical(float current_pressure, float roll, float pitch);
    void evaluateHorizontal();

    void writeMotors();

    int getTotalPower();
};


#endif
