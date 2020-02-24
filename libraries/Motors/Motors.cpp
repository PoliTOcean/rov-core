//#include "Arduino.h"
#include "Motors.h"

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

Motors::Motors(float dt)
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

void Motors::configure()
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
    setPower(power::SLOW);

    configured = true;
}

//function to evaluate vertical motors values
void Motors::evaluateVertical(float currentPressure, float roll, float pitch)
{
    float pitchPower, rollPower;
    if (!started)
    {
        upperRightTruster.stop();
        upperLeftTruster.stop();
        upperBackTruster.stop();
        return;
    }

    //value for up-down movement
    int valUD = 0;
    float depthCorrectionPower = 0;
    if (down > 0 || up > 0)
    {                                       //controlled up-down from joystick
        savePressure = true;                //it has to save pressure when finished
        valUD = (up - down) * DEF_AXIS_MAX; //fixed value depending on buttons pressed
    }
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
    upperLeftTruster.setValue(valUD, true);
    upperRightTruster.setValue(valUD, true);
    upperBackTruster.setValue(valUD, true);
}

void Motors::writeMotors()
{
    upperLeftTruster.write();
    upperRightTruster.write();
    upperBackTruster.write();
    frontLeftTruster.write();
    frontRightTruster.write();
    backLeftTruster.write();
    backRightTruster.write();
}

void Motors::evaluateHorizontal()
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
    int rz = this->rz;
    switch (powerMode)
    {
    case SLOW:
        rz = 0.8 * rz;
        break;
    case MEDIUM:
        rz = 0.6 * rz;
        break;
    case FAST:
        rz = 0.4 * rz;
        break;
    }

    // should we update?
    bool updateNow = timer.onRestart();

    // set values
    frontLeftTruster.setValue(signFL * (-y + x + rz), updateNow);
    frontRightTruster.setValue(signFR * (-y - x - rz), updateNow);
    int valueBL = signBL * (-y - x + rz);
    int valueBR = signBR * (-y + x - rz);
    backLeftTruster.setValue(valueBL, updateNow);
    backRightTruster.setValue(valueBR, updateNow);

    // if the rov request full power  for all its motors reduce the
    // value of BL and BR in a certain percentage in order to
    // prevent 12V power protection
    if (powerMode == power::FAST && getTotalPower() > PWR_THRESHOLD)
    {
        backLeftTruster.setValue(valueBL * PWR_CUT_PERC, true);
        backRightTruster.setValue(valueBR * PWR_CUT_PERC, true);
    }
}

void Motors::start()
{
    if (!configured)
        return;
    started = true;
    savePressure = true;
}

void Motors::stop()
{
    x = 0;
    y = 0;
    rz = 0;
    started = false;
}

void Motors::stopUp()
{
    up = 0;
}

void Motors::stopDown()
{
    down = 0;
}

void Motors::goUp()
{
    up = 0.6;
}

void Motors::goUpFast()
{
    up = 1;
}

void Motors::stopUpFast()
{
    up = 0;
}

void Motors::goDown()
{
    down = 0.8;
}

void Motors::setX(int x)
{
    if (x < DEF_AXIS_MIN)
        x = DEF_AXIS_MIN;
    else if (x > DEF_AXIS_MAX)
        x = DEF_AXIS_MAX;
    else
        this->x = x;
}

void Motors::setY(int y)
{
    if (y < DEF_AXIS_MIN)
        y = DEF_AXIS_MIN;
    else if (y > DEF_AXIS_MAX)
        y = DEF_AXIS_MAX;
    else
        this->y = y;
}

void Motors::setRz(int rz)
{
    if (rz < DEF_AXIS_MIN)
        rz = DEF_AXIS_MIN;
    else if (rz > DEF_AXIS_MAX)
        rz = DEF_AXIS_MAX;
    else
        this->rz = rz;
}

void Motors::setPower(power pwr)
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

int Motors::getTotalPower()
{
    return frontRightTruster.getValue() - SERVO_STOP_VALUE + frontLeftTruster.getValue() - SERVO_STOP_VALUE + backRightTruster.getValue() - SERVO_STOP_VALUE + backLeftTruster.getValue() - SERVO_STOP_VALUE + upperRightTruster.getValue() - SERVO_STOP_VALUE + upperLeftTruster.getValue() - SERVO_STOP_VALUE + upperBackTruster.getValue() - SERVO_STOP_VALUE;
}

void Motors::setPitchControlPower(int pitchPower)
{
    this->pitchControlPower = pitchPower;
}

void Motors::pitchControlOn()
{
    this->pitchControlEnabled = true;
}

void Motors::pitchControlOff()
{
    this->pitchControlEnabled = false;
}