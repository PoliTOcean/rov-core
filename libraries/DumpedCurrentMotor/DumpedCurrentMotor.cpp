#include "DumpedCurrentMotor.h"
#include "Servo.h"
#include "Arduino.h"
#define vmap(x, i_min, i_max, o_min, o_max) (long) (x - i_min) * (o_max - o_min) / ( i_max - i_min ) + o_min


/*
 * in the vect vector are saved all the instances of the motor that will be create in the following way
 * Motor code    |   0    |   1    |   2    |   3    |   4    |   5    |   6   |
 * index vector  |   0    |   1    |   2    |   3    |   4    |   5    |   6   |
 */
Motor* vect[MOTORS_N];
int i = 0;

// list management data and functions
void insert(int);
struct list_Motor
{
  int  p;
  list_Motor* s;
};
list_Motor* list = NULL;

bool Motor::timer_setup = false;

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

  if(!Motor::timer_setup){
    //Timer setup
    cli();
    TCCR2A = 0;                                    // set entire TCCR1A register to 0
    TCCR2B = 0;                                    // same for TCCR1B
    TCNT2  = 0;                                    // initialize counter value to 0
    OCR2A  = 255;                                  // COMPARE REGISTER A = (16*10^6) / (1*1024) - 1 (must be <256 -> 8 bit)-------> [16*10^6/(prescale*desired frequncy)] -1
    TCCR2B |= (1 << WGM22);                        // turn on CTC mode
    TCCR2B |= (1 << CS22) | (1 << CS20);           // Set CS12 and CS10 bits for 1024 prescaler
    TIMSK2 &= ~(1 << OCIE2A);
    sei();
    Motor::timer_setup = true;
  }
  this->pin = pin;
  this->motor.attach(this->pin);
  this->motor.writeMicroseconds(SERVO_STOP_VALUE);
}


/** detach the motor **/
void Motor::detach(){
  this->pin   = -1;
  this->value = SERVO_STOP_VALUE;
  
  this->motor.writeMicroseconds(SERVO_STOP_VALUE);
  this->motor.detach();
}


/** update function
 *
 *  Updates the current value and writes it to the motor.
 *
 *  @return true if the reach_value has been reached
 */
bool Motor::update()                    // update the current value by one step
{
  static int prev_offset = 0;

  int current_offset = offset - prev_offset;

  if (abs(this->value + current_offset - this->reach_value) <= this->step)
  {
    this->value = this->reach_value;
  }
  else if (this->value < this->reach_value)
  {
    this->value += this->step + current_offset;
  }
  else if (this->value > this->reach_value)
  {
    this->value -= this->step + current_offset;
  }

  prev_offset = offset;

  if (SERVO_STOP_THRESHOLD_MIN < this->value && this->value < SERVO_STOP_THRESHOLD_MAX)
    this->value = SERVO_STOP_VALUE;

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


/** Method to set the reach_value
 *
 *  @param val
 *
 *  @return void
 */
void Motor::set_value(int val)                                       // set the new value of the current to reach and the step
{
  if (val > input_maxval) this->reach_value = maxval;                        // saturation max value
  else if (val < input_minval) this->reach_value = minval;                   // stauration min value
  else 
    this->reach_value = (long) (val - input_minval) * (maxval - minval) / (input_maxval - input_minval) + minval;
  
  this->reach_value += offset;
  
  if (!this->is_value_reached()){
    insert(this->code);
    TIMSK2 |= (1 << OCIE2A);
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
  step = (maxval - minval) * stepPerc / 100;

  //functional values setup
  this->value       = SERVO_STOP_VALUE;
  this->reach_value = this->value;
  this->pin = -1;

  //final motor setup
  this->code        = i;
  vect[i] = this;
  i++;
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

int Motor::get_code()
{
  return this->code;
}


// ---------- Management of the list below: ----------
/** Timer interrupt handler:
 *  Checks the list. If not empty, updates the value of the next motor.
 *  If it has not reached the value (if so, method 'update' returns true),
 *  push the motor into the list again.
 */
ISR(TIMER2_COMPA_vect)
{
  //CHECK THE LIST
  if (list != NULL)
  {
    list_Motor* app;
    app         = list;
    if (!vect[list->p]->update()) insert(list->p);
    list        = app->s;
    delete(app);
  }
  else
  {
      TIMSK2 &= (0 << OCIE2A);
  }
}

/** insert motor into the list
 *
 *  @param code of the motor
 *
 *  @return void
 */
void insert(int code)
{
  list_Motor* z;
  if(list == NULL)
  {
    list = new(list_Motor);
    list->p = code;
    list->s = NULL;
  }
  else
  {
    z = list;
    while(z->s != NULL) z = z->s;
    z->s = new(list_Motor);
    z = z->s;
    z->p = code;
    z->s = NULL;
  }
}
