#include "Truster.h"

#define vmap(x, i_min, i_max, o_min, o_max) (long)(x - i_min) * (o_max - o_min) / (i_max - i_min) + o_min

Truster::Truster(int minInputVal, int maxInputVal, int offsetPower, int startPowerPerc, int stepPerc)
    : minInputVal(minInputVal), maxInputVal(maxInputVal)
{
    setOffsetPower(offsetPower);
    offset = 0;
    setPower(startPowerPerc);

    if (stepPerc < 0)
        stepPerc = 0;
    else if (stepPerc > 100)
        stepPerc = 100;

    step = MAX_POWER * stepPerc / 100;

    value = SERVO_STOP_VALUE;
    reachValue = value;
    pin = -1;
}

void Truster::attach(int pin)
{
    if (pin <= 0 || pin > MAX_PIN)
        return;

    this->pin = pin;
    motor.attach(pin);
    motor.writeMicroseconds(SERVO_STOP_VALUE);
}

void Truster::detach()
{
    pin = -1;
    stop();
    motor.detach();
}

void Truster::write()
{
    value = reachValue;
    motor.writeMicroseconds(value);
}

bool Truster::update()
{
    int currentOffset = 0;

    if (reachValue > SERVO_STOP_VALUE && value + step + currentOffset > reachValue ||
        reachValue < SERVO_STOP_VALUE && value - step + currentOffset < reachValue ||
        (value < SERVO_STOP_VALUE && reachValue <= SERVO_STOP_VALUE && value < reachValue) ||
        (value > SERVO_STOP_VALUE && reachValue >= SERVO_STOP_VALUE && value > reachValue))
    {
        // if its intensity is going to 0, or if the step is closer enough, then set to reach_value
        value = reachValue;
    }
    else if (value < SERVO_STOP_VALUE && reachValue >= SERVO_STOP_VALUE ||
             value > SERVO_STOP_VALUE && reachValue <= SERVO_STOP_VALUE)
    {
        // if its intensity is going to 0, but reach_value has changed sign, then set to 0
        value = SERVO_STOP_VALUE;
    }
    else if (value < reachValue)
    {
        // if intensity is growing up, approach to reach_value by one step
        value += step;
    }
    else if (value > reachValue)
    {
        value -= step;
    }

    motor.writeMicroseconds(value);

    return isValueReached();
}

void Truster::stop()
{
    offset = 0;
    reachValue = SERVO_STOP_VALUE;
    value = reachValue;

    motor.writeMicroseconds(value);
}

bool Truster::isValueReached()
{
    return value == reachValue;
}

void Truster::setValue(int val, bool toUpdate)
{
    if (val > maxInputVal)
        reachValue = maxVal; // saturation max value
    else if (val < minInputVal)
        reachValue = minVal; // stauration min value
    else
        reachValue = (long)(val - minInputVal) * (maxVal - minVal) / (maxInputVal - minInputVal) + minVal;

    reachValue += offset;

    if (toUpdate)
        update();
}

void Truster::setOffset(int offset, bool toUpdate = false)
{
    if (offset > maxInputVal)
        this->offset = maxOffsetVal; // saturation max value
    else if (offset < minInputVal)
        this->offset = minOffsetVal; // stauration min value
    else
        this->offset = vmap(offset, minInputVal, maxInputVal, minOffsetVal, maxOffsetVal);

    if (toUpdate)
        setValue(value, true);
}

void Truster::setOffsetPower(int powerPerc)
{
    int power = 0;
    if (powerPerc < 0)
        powerPerc = 0;
    else if (powerPerc > MAX_OFFSET_PERC)
        powerPerc = MAX_OFFSET_PERC;

    power = powerPerc * MAX_POWER / 100;

    minOffsetVal = -power;
    maxOffsetVal = power;
}

void Truster::setPower(int powerPerc)
{
    if (powerPerc < 0)
        powerPerc = 0;
    else if (powerPerc > 100)
        powerPerc = 100;

    int power = powerPerc * MAX_POWER / 100;

    minVal = SERVO_STOP_VALUE - power;
    maxVal = SERVO_STOP_VALUE + power;
}

int Truster::getPin()
{
    return pin;
}

int Truster::getValue()
{
    return value;
}

int Truster::getMinVal()
{
    return minVal;
}
int Truster::getMaxVal()
{
    return maxVal;
}

int Truster::getReachValue()
{
    return reachValue;
}

int Truster::getOffset()
{
    return offset;
}

int Truster::getStep()
{
    return step;
}
