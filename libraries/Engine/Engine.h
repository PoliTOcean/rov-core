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

#define DEF_AXIS_MIN_H -126
#define DEF_AXIS_MAX_H 127
#define DEF_AXIS_MIN_V 0
#define DEF_AXIS_MAX_V 255

/*
#define DEF_AXIS_MIN -126
#define DEF_AXIS_MAX 127
*/
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

class Engine
{
    const float horizontalPowerPerc[3] = {H_SLOW_POWER, H_MEDIUM_POWER, H_FAST_POWER};
    const float verticalPowerPerc[3] = {V_SLOW_POWER, V_MEDIUM_POWER, V_FAST_POWER};

    int x, y, yaw;
    float up, down;

    float requestedPressure;
    int pressCount;

    int pitchControlPower;

    bool savePressure, started, configured = false;
    bool pitchControlEnabled;

    long long startTimeForPressure;

    Truster frontLeftTruster,
        frontRightTruster,
        backLeftTruster,
        backRightTruster,
        upperRightTruster,
        upperLeftTruster,
        upperBackTruster;

    PIDController pitchCorrection, rollCorrection, depthCorrection;

    RBD::Timer timer;

public:
    enum PowerMode
    {
        SLOW,
        MEDIUM,
        FAST
    };

    Engine(float dt);

    void configure();

    void start();
    void stop();

    void setX(int x);
    void setY(int y);
    void setYaw(int yaw);

    void setUp(int up);
    void setDown(int down);

    void pitchControlOn();
    void pitchControlOff();
    void setPitchControlPower(int pitchPower);

    void setPower(PowerMode pwr);

    void evaluateVertical(float currentPressure, float roll, float pitch);
    void evaluateHorizontal();

    void writeMotors();

    int getTotalPower();

    bool isStarted();
    bool isPitchControlEnabled();

private:
    PowerMode powerMode;
};

#endif
