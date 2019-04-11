#include "Arduino.h"
#include "Servo.h"
#ifndef _LIMIT_CURRENT_H
#define _LIMIT_CURRENT_H

//constants

#define DEFAULT_PERC 10                                                 // default percentage of the step
#define DEFAULT_MAX_PERC 30                                             // default max for the percentage for the step          
#define DEFAULT_MAX_VAL 1900					 	                                // default max value for the actual value
#define DEFAULT_MIN_VAL 1100                                            // default min value for the actual value
#define DEFAULT_INT_VAL 15624                                            // default value for compare register  

//DumpedCurrentMotor class with variable step

class Motor {
   private:
      volatile int value;                                            		      // actual value of the current
      volatile int reach_value;                                      		      // value to reach
      volatile int step;						                                          // step of the DumpedCurrentMotor
      volatile int maxval;                                           		      // maximum value of the DumpedCurrentMotor
      volatile int minval;                                                    // minimum value of the DumpedCurrentMotor
      volatile int code;                                                      // univoque value of the motor
      volatile Servo motor;
   public:                           		      
      Motor(int pin);
      Motor(int stepp, int pin);
      Motor(int maxval, int minval, int pin);
      Motor(int minim,int maxim, int perc, int pin);
      Motor(int minim,int maxim, int perc, int int_val, int pin);
      void set_value(int val);
      bool update();
      void insert();
      bool is_value_reached();
      int get_value();
      int get_reach_value();
      int get_step();
      int get_maxval();
      int get_minval();
      int get_code();

   protected:
         void init(int, int, int, int);
 
};

#endif
