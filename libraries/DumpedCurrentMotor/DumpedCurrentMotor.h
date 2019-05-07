#include "Arduino.h"
#include "Servo.h"
#ifndef POLITOCEAN_LIMIT_CURRENT_H
#define POLITOCEAN_LIMIT_CURRENT_H

#define DEFAULT_POWER      150
#define MAX_POWER          400

//useful constants
#define DEFAULT_PERC       10                                        // default percentage of the step
#define DEFAULT_MAX_PERC   30                                        // default max for the percentage for the step          
#define SERVO_STOP_VALUE   1500                                      // default servo stop value
#define DEFAULT_MAX_VAL    SERVO_STOP_VALUE+DEFAULT_POWER				// default max value for the actual value
#define DEFAULT_MIN_VAL    SERVO_STOP_VALUE-DEFAULT_POWER            // default min value for the actual value
#define DEFAULT_INT_VAL    15624                                     // default value for compare register  
#define MAX_PIN            13                                        // max pin attachable to the motor

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
      /*volatile*/ int value;                                            		    // current value
      /*volatile*/ int reach_value;                                      		    // value to reach
      /*volatile*/ int step;						                                     // step of the DumpedCurrentMotor
      /*volatile*/ int maxval;                                           		    // maximum value of the DumpedCurrentMotor
      /*volatile*/ int minval;                                                    // minimum value of the DumpedCurrentMotor
      /*volatile*/ int code;                                                      // univoque value of the motor
      /*volatile*/ int pin;                                                       // pin where the motor is attached to
      /*volatile*/ Servo motor;                                                   // Servo instance
      /*volatile*/ int in_minval;
      /*volatile*/ int in_maxval;

   public:
      Motor();
      
      void set_value(int val);
      bool is_value_reached();
      int get_value();
      int get_reach_value();
      int get_step();
      int get_maxval();
      int get_minval();
      int get_code();
      int get_pin();
      void init(int in_minval, int in_maxval, int power/*, int perc*/);
      void attach(int pin);
      void detach();
      bool update();
 
};

#endif
