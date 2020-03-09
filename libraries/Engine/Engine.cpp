//#include "Arduino.h"
#include "Engine.h"

#define PWR_CUT_PERC 0.4                   // 40%
#define PWR_THRESHOLD MAX_POWER * 7 * 0.75 // 90% of total power

#define N_PRESS_SAMPLES 5
#define N_SECS 3
#define PRESS_TIME_THRESHOLD N_SECS * 1000000 / N_PRESS_SAMPLES //us

namespace Pinout
{
static const int UpperRightTruster = 8;
static const int UpperLeftTruster = 7;
static const int UpperBackTruster = 4;
static const int FrontRightTruster = 5;
static const int FrontLeftTruster = 6;
static const int BackRightTruster = 3;
static const int BackLeftTruster = 2;
}; // namespace Pinout

namespace Signs
{
const int signFrontLeft = -1;
const int signFrontRight = -1;
const int signBackLeft = 1;
const int signBackRight = 1;
const int signUpperRight = 1;
const int signUpperLeft = 1;
const int signUpperBack = 1;
} // namespace Signs

Engine::Engine(float dt)
    : frontLeftTruster(DEF_AXIS_MIN, DEF_AXIS_MAX, 0, 0, H_POWER_STEP),
      frontRightTruster(DEF_AXIS_MIN, DEF_AXIS_MAX, 0, 0, H_POWER_STEP),
      backLeftTruster(DEF_AXIS_MIN, DEF_AXIS_MAX, 0, 0, H_POWER_STEP),
      backRightTruster(DEF_AXIS_MIN, DEF_AXIS_MAX, 0, 0, H_POWER_STEP),
      upperLeftTruster(DEF_AXIS_MIN, DEF_AXIS_MAX, V_OFFSET_POWER, 0, V_POWER_STEP),
      upperRightTruster(DEF_AXIS_MIN, DEF_AXIS_MAX, V_OFFSET_POWER, 0, V_POWER_STEP),
      upperBackTruster(DEF_AXIS_MIN, DEF_AXIS_MAX, V_OFFSET_POWER, 0, V_POWER_STEP),
      pitchCorrection(KP_pitch, KI_pitch, KD_pitch, dt, DEF_AXIS_MAX),
      rollCorrection(KP_roll, KI_roll, KD_roll, dt, DEF_AXIS_MAX),
      depthCorrection(KP_depth, KI_depth, KD_depth, dt, DEF_AXIS_MAX),
      pitchControlEnabled(false)
{
    timer.setTimeout(DEF_TIME_TO_UPDATE_MS);
    timer.restart();
}

void Engine::configure()
{
    // attach motors
    upperRightTruster.attach(Pinout::UpperRightTruster);
    upperLeftTruster.attach(Pinout::UpperLeftTruster);
    upperBackTruster.attach(Pinout::UpperBackTruster);
    frontRightTruster.attach(Pinout::FrontRightTruster);
    frontLeftTruster.attach(Pinout::FrontLeftTruster);
    backRightTruster.attach(Pinout::BackRightTruster);
    backLeftTruster.attach(Pinout::BackLeftTruster);

    stop(); // do not run the motors untill `start()` is called
    savePressure = false;
    pitchControlOff();
    setPower(PowerMode::SLOW);

    configured = true;
}

//function to evaluate vertical motors values
void Engine::evaluateVertical(float currentPressure, float roll, float pitch)
{
    float pitchPower = 0, rollPower = 0;
    if (!started)
    {
        upperRightTruster.stop();
        upperLeftTruster.stop();
        upperBackTruster.stop();
        return;
    }

    //value for up-down movement
    float depthCorrectionPower = 0;
    if (z != 0)
        savePressure = true; //it has to save pressure when finished
    else
    {
        if (savePressure)
        {
            pressCount = 0;
            savePressure = false;
        }
        else if (pressCount < N_PRESS_SAMPLES && micros() - startTimeForPressure > (long)PRESS_TIME_THRESHOLD)
        {
            requestedPressure = currentPressure;
            startTimeForPressure = micros();
            depthCorrection.reset();
            pressCount++;
        }
        depthCorrectionPower = -depthCorrection.calculate_power(currentPressure, requestedPressure);
    }

    if (pitchControlEnabled)
    {
        pitchPower = pitchControlPower;
        depthCorrectionPower = 0;
        rollPower = 0;
        depthCorrection.reset();
        pitchCorrection.reset();
    }
    else
    {
        pitchPower = pitchCorrection.calculate_power(pitch, 0.02);
        rollPower = rollCorrection.calculate_power(roll, 0);
    }

    //adding values for UD movement/autoquote
    upperLeftTruster.setOffset(depthCorrectionPower + pitchPower + rollPower);
    upperRightTruster.setOffset(depthCorrectionPower + pitchPower - rollPower);
    upperBackTruster.setOffset(depthCorrectionPower - 2 * pitchPower);
    upperLeftTruster.setValue(z, true);
    upperRightTruster.setValue(z, true);
    upperBackTruster.setValue(z, true);
}

void Engine::writeMotors()
{
    upperLeftTruster.write();
    upperRightTruster.write();
    upperBackTruster.write();
    frontLeftTruster.write();
    frontRightTruster.write();
    backLeftTruster.write();
    backRightTruster.write();
}

void Engine::evaluateHorizontal()
{
    if (!started)
    {
        frontLeftTruster.stop();
        frontRightTruster.stop();
        backLeftTruster.stop();
        backRightTruster.stop();
        return;
    }

    //rotation inibition
    int yaw = this->yaw;
    switch (powerMode)
    {
    case SLOW:
        yaw = 0.8 * yaw;
        break;
    case MEDIUM:
        yaw = 0.6 * yaw;
        break;
    case FAST:
        yaw = 0.4 * yaw;
        break;
    }

    // should we update?
    bool updateNow = timer.onRestart();

    // set values
    frontLeftTruster.setValue(Signs::signFrontLeft * (-y + x + yaw), updateNow);
    frontRightTruster.setValue(Signs::signFrontRight * (-y - x - yaw), updateNow);
    int valueBL = Signs::signBackLeft * (-y - x + yaw);
    int valueBR = Signs::signBackRight * (-y + x - yaw);
    backLeftTruster.setValue(valueBL, updateNow);
    backRightTruster.setValue(valueBR, updateNow);

    // if the rov request full power  for all its motors reduce the
    // value of BL and BR in a certain percentage in order to
    // prevent 12V power protection
    if (powerMode == PowerMode::FAST && getTotalPower() > PWR_THRESHOLD)
    {
        backLeftTruster.setValue(valueBL * PWR_CUT_PERC, true);
        backRightTruster.setValue(valueBR * PWR_CUT_PERC, true);
    }
}

void Engine::start()
{
    if (!configured)
        return;
    started = true;
    savePressure = true;
}

void Engine::stop()
{
    x = 0;
    y = 0;
    z = 0;
    yaw = 0;

    started = false;
}

void Engine::setX(int x)
{
    if (x < DEF_AXIS_MIN)
        this->x = DEF_AXIS_MIN;
    else if (x > DEF_AXIS_MAX)
        this->x = DEF_AXIS_MAX;
    else
        this->x = x;
}

void Engine::setY(int y)
{
    if (y < DEF_AXIS_MIN)
        this->y = DEF_AXIS_MIN;
    else if (y > DEF_AXIS_MAX)
        this->y = DEF_AXIS_MAX;
    else
        this->y = y;
}

void Engine::setYaw(int yaw)
{
    if (yaw < DEF_AXIS_MIN)
        this->yaw = DEF_AXIS_MIN;
    else if (yaw > DEF_AXIS_MAX)
        this->yaw = DEF_AXIS_MAX;
    else
        this->yaw = yaw;
}

void Engine::setZ(int z)
{
    if (z < DEF_AXIS_MIN)
        this->z = DEF_AXIS_MIN;
    else if (z > DEF_AXIS_MAX)
        this->z = DEF_AXIS_MAX;
    else
        this->z = z;
}

void Engine::setPower(Engine::PowerMode pwr)
{
    int perc = horizontalPowerPerc[static_cast<int>(pwr)];
    frontRightTruster.setPower(perc);
    frontLeftTruster.setPower(perc);
    backRightTruster.setPower(perc);
    backLeftTruster.setPower(perc);

    perc = verticalPowerPerc[static_cast<int>(pwr)];
    upperRightTruster.setPower(perc);
    upperLeftTruster.setPower(perc);
    upperBackTruster.setPower(perc);

    this->powerMode = pwr;
}

int Engine::getTotalPower()
{
    return frontRightTruster.getValue() - SERVO_STOP_VALUE + frontLeftTruster.getValue() - SERVO_STOP_VALUE + backRightTruster.getValue() - SERVO_STOP_VALUE + backLeftTruster.getValue() - SERVO_STOP_VALUE + upperRightTruster.getValue() - SERVO_STOP_VALUE + upperLeftTruster.getValue() - SERVO_STOP_VALUE + upperBackTruster.getValue() - SERVO_STOP_VALUE;
}

void Engine::setPitchControlPower(int pitchPower)
{
    this->pitchControlPower = pitchPower;
}

void Engine::pitchControlOn()
{
    this->pitchControlEnabled = true;
}

void Engine::pitchControlOff()
{
    this->pitchControlEnabled = false;
}

bool Engine::isStarted()
{
    return started;
}

bool Engine::isPitchControlEnabled()
{
    return pitchControlEnabled;
}