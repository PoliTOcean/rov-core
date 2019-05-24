#include "Arduino.h"
#include "Servo.h"
#ifndef POLITOCEAN_LIMIT_CURRENT_H
#define POLITOCEAN_LIMIT_CURRENT_H

#define MAX_POWER                   400
#define MAX_OFFSET_PERC             50

//useful constants
#define DEF_POWER_PERC              30
#define DEF_STEP_PERC               10                                        // default percentage of the step
#define DEFAULT_MAX_VAL             1700					 	                     // default max value for the actual value
#define DEFAULT_MIN_VAL             1300                                      // default min value for the actual value
#define DEFAULT_INT_VAL             15624                                     // default value for compare register  
#define SERVO_STOP_VALUE            1500                                      // default servo stop value
#define SERVO_STOP_THRESHOLD_MIN    SERVO_STOP_VALUE - SERVO_STOP_THRESHOLD
#define SERVO_STOP_THRESHOLD_MAX    SERVO_STOP_VALUE + SERVO_STOP_THRESHOLD
#define MAX_PIN           13                                                  // max pin attachable to the motor

//constant to define number of motors
#define MOTORS_N          7

/** DumpedCurrentMotor class 
 *  
 *  Wrapper class for the Servo motor, which provides the developer with a variable step, which is used to update
 *  the servo speed gradually using a timer.
 *  It's made to handle 7 motors (it can be edited to handle more)
 */#define SERVO_STOP_THRESHOLD        50

class Motor {
   private:
      int value;                                   // current value
      int reach_value;                             // value to reach
      int step;						                  // step of the DumpedCurrentMotor
      Servo motor;                                 // Servo instance
      int offset;                                  // offset value
      int prev_offset;
      int pin;                                     // pin where the motor is attached to
      int maxval;                                  // maximum value of the DumpedCurrentMotor
      int minval;                                  // minimum value of the DumpedCurrentMotor
      int input_minval;                            // minimum value of the input power
      int input_maxval;                            // maximum value of the input power
      int offset_minval;                           // minimum value of the offset
      int offset_maxval;                           // maximum value of the offset

   public:
      static bool timer_setup;
      Motor(int in_min, int in_max, int offsetPower = 0, int startPowerPerc = DEF_POWER_PERC, int stepPerc = DEF_STEP_PERC);
      
      void set_offset_power(int powerPerc);
      void set_offset(int offset);
      int get_offset();
      void set_value(int val, bool to_update = false);
      void set_and_update(int offset, int value);
      bool is_value_reached();
      int get_value();
      int get_reach_value();
      int get_step();
      int get_maxval();
      int get_minval();
      int get_pin();
      void attach(int pin);
      void detach();
      bool update();
      void set_power(int powerPerc);
      void write();
      void stop();
};

#endif
