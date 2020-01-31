#include "Servo.h"

#ifndef POLITOCEAN_TRUSTER_H
#define POLITOCEAN_TRUSTER_H

#define MAX_POWER 400
#define MAX_OFFSET_PERC 80

#define DEF_POWER_PERC 30
#define DEF_STEP_PERC 10      ///< Default percentage of the step
#define DEFAULT_MIN_VAL 1300  ///< Default min value for the actual value
#define DEFAULT_MAX_VAL 1700  ///< Default max value for the actual value
#define DEFAULT_INT_VAL 15624 ///< Default value for compare register
#define SERVO_STOP_VALUE 1500 ///< Default servo stop value
#define SERVO_STOP_THRESHOLD 50
#define SERVO_STOP_THRESHOLD_MIN SERVO_STOP_VALUE - SERVO_STOP_THRESHOLD
#define SERVO_STOP_THRESHOLD_MAX SERVO_STOP_VALUE + SERVO_STOP_THRESHOLD
#define MAX_PIN 13 ///< Max pin attachable to the motor

/** 
 * Wrapper class for the Servo motor
 * 
 * It provides the developer with a variable step, which is used to update the servo speed gradually using a timer.
 */

class Truster
{
private:
   Servo motor; ///< Servo instance

   int value;        ///< Current value
   int reachValue;   ///< Value to reach
   int step;         ///< Step of the current dump
   int offset;       ///< Offset value
   int prevOffset;   ///< Previous offset value
   int pin;          ///< Pin where the motor is attached to
   int maxVal;       ///< Maximum value of the Truster
   int minVal;       ///< Minimum value of the Truster
   int minInputVal;  ///< Minimum value of the input power
   int maxInputVal;  ///< Maximum value of the input power
   int minOffsetVal; ///< Minimum value of the offset
   int maxOffsetVal; ///< Maximum value of the offset

public:
   Truster(int minInputVal, int maxInputVal, int offsetPower = 0, int startPowerPerc = DEF_POWER_PERC, int stepPerc = DEF_STEP_PERC);

   /**
    * Attach the motor to the pin
    * 
    * @param pin contains the pin value to attach the motor
    */
   void attach(int pin);
   /**
    * Detach the motor from the pin
    */
   void detach();
   /**
    * Write the value to the motor
    */
   void write();
   /**
    * Update the current value by one step
    */
   bool update();
   /**
    * Stop the motor
    */
   void stop();

   /**
    * @return true if the value reached reachValue
    */
   bool isValueReached();

   /**
    * @param val the value to set
    * @param toUpdated (default to false) if the value must be updated by the truster
    */
   void setValue(int val, bool toUpdate = false);
   /**
    * @param offset the offset to set
    * @param toUpdate (default to false) if the value must be updated by the truster after setting the offset
    * 
    * TODO: Remove toUpdate, it may be useless
    */
   void setOffset(int offset, bool toUpdate = false);
   /**
    * @param powerPerc power percentage to determine minOffsetVal and maxOffsetVal
    */
   void setOffsetPower(int powerPerc);
   /**
    * @param powerPerc power perentage to determine minVal and maxVal
    */
   void setPower(int powerPerc);

   /**
    * @return the pin which the truster is attached to
    */
   int getPin();
   /**
    * @return the current value of the truster
    */
   int getValue();
   /**
    * @return the minimum value
    */
   int getMinVal();
   /**
    * @return the maximum value
    */
   int getMaxVal();
   /**
    * @return the value to reach
    */
   int getReachValue();
   /**
    * @return the offset
    */
   int getOffset();
   /**
    * @return the step
    */
   int getStep();
};

#endif // POLITOCEAN_TRUSTER_H