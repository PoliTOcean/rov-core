#include "Arduino.h"
#include "Servo.h"
#ifndef _LIMIT_CURRENT_H
#define _LIMIT_CURRENT_H

//useful constants
#define DEFAULT_PERC      10                                        // default percentage of the step
#define DEFAULT_MAX_PERC  30                                        // default max for the percentage for the step          
#define DEFAULT_MAX_VAL   1900					 	                          // default max value for the actual value
#define DEFAULT_MIN_VAL   1100                                      // default min value for the actual value
#define DEFAULT_INT_VAL   15624                                     // default value for compare register  
#define SERVO_STOP_VALUE  1500                                      // default servo stop value
#define MAX_PIN           13                                        // max pin attachable to the motor

//constant to define number of motors
#define MOTORS_N          7

/** DumpedCurrentMotor class 
 *  
 *  Wrapper class for the Servo motor, which provides the developer with a variable step, which is used to update
 *  the servo speed gradually using a timer.
 *  It's made to handle 7 motors (it can be edited to handle more.
 */
class Motor {
   private:
      volatile int value;                                            		      // current value
      volatile int reach_value;                                      		      // value to reach
      volatile int step;						                                          // step of the DumpedCurrentMotor
      volatile int maxval;                                           		      // maximum value of the DumpedCurrentMotor
      volatile int minval;                                                    // minimum value of the DumpedCurrentMotor
      volatile int code;                                                      // univoque value of the motor
      volatile int pin;                                                       // pin where the motor is attached to
      volatile Servo motor;                                                   // Servo instance

   public:                           		      
      Motor();
      Motor(int stepp);
      Motor(int maxval, int minval);
      Motor(int minim,int maxim, int stepp);
      
      void set_value(int val);
      bool is_value_reached();
      int get_value();
      int get_reach_value();
      int get_step();
      int get_maxval();
      int get_minval();
      int get_code();
      int get_pin();
      void attach(int pin);
      void detach();
      bool update();
      
   protected:
      void init(int, int, int);
 
};

#endif
