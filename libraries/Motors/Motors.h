/**
 * Motors sensor
 * PolitTOcean year 2019
 */

#ifndef POLITOCEAN_MOTORS_H
#define POLITOCEAN_MOTORS_H

#include "Truster.h"
#include "PressureSensor.h"
#include "IMU.h"
#include "PIDController.h"
#include "RBD_Timer.h"

/*
#define DEF_AXIS_MIN_H -126
#define DEF_AXIS_MAX_H 127
#define DEF_AXIS_MIN_V 0
#define DEF_AXIS_MAX_V 255
*/

#define DEF_AXIS_MIN -126
#define DEF_AXIS_MAX 127

#define V_OFFSET_POWER 80

#define TIME_TO_REACH_MAX 3                                        //seconds
#define DEF_TIME_TO_UPDATE_MS TIME_TO_REACH_MAX *H_POWER_STEP * 10 // time*h_power_perc/100 * 1000
#define H_POWER_STEP 1
#define H_SLOW_POWER 30
#define H_MEDIUM_POWER 50
#define H_FAST_POWER 90

#define V_POWER_STEP 1
#define V_SLOW_POWER 30
#define V_MEDIUM_POWER 80
#define V_FAST_POWER 100

#define KP_roll 60
#define KI_roll 0
#define KD_roll 0

#define KU_pitch 90
#define PU_pitch 1.57

#define KP_pitch 60 //0.2*KU_pitch          //0.6*KU_pitch
#define KI_pitch 0  //0.4*KU_pitch/PU_pitch //2*KP_pitch/PU_pitch
#define KD_pitch 0  //KU_pitch*PU_pitch/15  //KP_pitch*PU_pitch/8

#define KU_depth 60
#define PU_depth 2.54

#define KP_depth 0.6 * KU_depth
#define KI_depth 2 * KP_depth / PU_depth
#define KD_depth KP_depth *PU_depth / 8

class Motors
{
protected:
    const int signFL = -1;
    const int signFR = -1;
    const int signBL = 1;
    const int signBR = 1;
    const int signUR = 1;
    const int signUL = 1;
    const int signUB = 1;

    const float horizontalPowerPerc[3] = {H_SLOW_POWER, H_MEDIUM_POWER, H_FAST_POWER};
    const float verticalPowerPerc[3] = {V_SLOW_POWER, V_MEDIUM_POWER, V_FAST_POWER};

    volatile int x, y, rz, pitchControlPower;
    volatile bool savePressure;
    long long startTimeForPressure;
    float requestedPressure;
    volatile float axis_min, axis_max;
    Truster frontLeftTruster,
        frontRightTruster,
        backLeftTruster,
        backRightTruster,
        upperRightTruster,
        upperLeftTruster,
        upperBackTruster;
    PIDController pitchCorrection, rollCorrection, depthCorrection;
    RBD::Timer timer;
    int pressCount;

public:
    enum power
    {
        SLOW,
        MEDIUM,
        FAST
    };

    volatile bool started, pitchControlEnabled;
    volatile float up, down;
    volatile Motors::power powerMode;
    bool configured = false;

    Motors(float dt);

    void configure();

    void start();
    void stop();

    void setX(int x);
    void setY(int y);
    void setRz(int rz);
    void setPitchControlPower(int pitchPower);

    void pitchControlOn();
    void pitchControlOff();
    void stopUp();
    void stopDown();
    void goUp();
    void goDown();
    void goUpFast();
    void stopUpFast();

    void setPower(Motors::power pwr);

    void evaluateVertical(float currentPressure, float roll, float pitch);
    void evaluateHorizontal();

    void writeMotors();

    int getTotalPower();
};

#endif
