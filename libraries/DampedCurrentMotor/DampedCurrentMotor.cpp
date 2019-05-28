#include "DampedCurrentMotor.h"
#include "Servo.h"
#include "Arduino.h"
#define vmap(x, i_min, i_max, o_min, o_max) (long) (x - i_min) * (o_max - o_min) / ( i_max - i_min ) + o_min

/**
 * set power function
 */
void Motor::set_power(int powerPerc){
  int power = 0;
  if(powerPerc < 0) powerPerc = 0;
  else if(powerPerc > 100) powerPerc = 100;

  power = powerPerc*MAX_POWER/100;

  this->minval = SERVO_STOP_VALUE - power;
  this->maxval = SERVO_STOP_VALUE + power;
}


/** attach the motor to a pin
 *
 *  @param pin
 *
 *  @return void
 */
void Motor::attach(int pin){
  if(pin<=0 || pin>MAX_PIN) return;

  this->pin = pin;
  this->motor.attach(this->pin);
  this->motor.writeMicroseconds(SERVO_STOP_VALUE);
}


/** detach the motor **/
void Motor::detach(){
  this->pin   = -1;
  stop();
  this->motor.detach();
}

void Motor::stop(){
  this->offset = 0;
  this->reach_value = SERVO_STOP_VALUE;
  this->value = this->reach_value;
  this->motor.writeMicroseconds(this->value);
}

void Motor::write(){
  this->value = this->reach_value;
  this->motor.writeMicroseconds(this->value);
}

/** update function
 *
 *  Updates the current value and writes it to the motor.
 *
 *  @return true if the reach_value has been reached
 */
bool Motor::update()                    // update the current value by one step
{
  int current_offset = offset - prev_offset;

  if (   this->value + this->step + current_offset > this->reach_value
      || this->value - this->step + current_offset < this->reach_value )
  {
    this->value = this->reach_value;
  }
  else if (this->value < this->reach_value)
  {
    this->value += this->step;
    this->value += current_offset;
  }
  else if (this->value > this->reach_value)
  {
    this->value -= this->step;
    this->value += current_offset;
  }

  prev_offset = offset;

  this->motor.writeMicroseconds(this->value);

  return this->is_value_reached();
}

/** Method to set the offset power
 * 
 * @param offset power percentage
 * 
 * @return void
 */
void Motor::set_offset_power(int powerPerc){
  int power = 0;
  if(powerPerc < 0) powerPerc = 0;
  else if(powerPerc > MAX_OFFSET_PERC) powerPerc = MAX_OFFSET_PERC;

  power = powerPerc*MAX_POWER/100;

  this->offset_minval = -power;
  this->offset_maxval =  power;
}

/** Method to set the offset value
 * 
 * @param offset
 * 
 * @return void
 */
void Motor::set_offset(int offset)
{
  if (offset > input_maxval) this->offset = offset_maxval;                        // saturation max value
  else if (offset < input_minval) this->offset = offset_minval;                   // stauration min value
  else this->offset = vmap(offset, input_minval, input_maxval, offset_minval, offset_maxval);
}

void Motor::set_and_update(int offset, int value){
  set_offset(offset);
  set_value(value, true);
}

/** Method to set the reach_value
 *
 *  @param val
 *
 *  @return void
 */
void Motor::set_value(int val, bool to_update)                                       // set the new value of the current to reach and the step
{
  if (val > input_maxval) this->reach_value = maxval;                        // saturation max value
  else if (val < input_minval) this->reach_value = minval;                   // stauration min value
  else 
    this->reach_value = (long) (val - input_minval) * (maxval - minval) / (input_maxval - input_minval) + minval;
  
  this->reach_value += offset;

  if(to_update)
  {
    update();
  }
}


/** method to check if the final value has been reached **/
bool Motor::is_value_reached(){
  return this->value == this->reach_value;
}


/** constructor **/
Motor::Motor(int in_min, int in_max, int offsetPower, int startPowerPerc, int stepPerc)
    : input_minval(in_min), input_maxval(in_max)
{
  set_offset_power(offsetPower);
  this->offset = 0;
  set_power(startPowerPerc);

  if(stepPerc < 0) stepPerc = 0;
  else if(stepPerc > 100) stepPerc = 100;
  step = MAX_POWER * stepPerc / 100;

  //functional values setup
  this->value       = SERVO_STOP_VALUE;
  this->reach_value = this->value;
  this->pin         = -1;
}

/** getters **/
int Motor::get_pin(){
  return this->pin;
}

int Motor::get_offset(){
  return this->offset;
}

int Motor::get_reach_value()
{
  return this->reach_value;
}

int Motor::get_maxval()
{
  return this->maxval;
}

int Motor::get_minval()
{
  return this->minval;
}

int Motor::get_step()
{
  return this->step;
}

int Motor::get_value()
{
  return this->value;
}
