#include "DumpedCurrentMotor.h"
#include "Servo.h"
#include "Arduino.h"

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

/** init function:
 *  sets max and min values, as well as the percentage to calculate the step
 *
 *  @param max value
 *  @param min value
 *  @param power
 *  @param percentage of step
 *
 *  @return void
 */
void Motor::init(int in_minval, int in_maxval){
  init(in_minval, in_maxval, DEFAULT_POWER, DEFAULT_PERC);
}

void Motor::init(int in_minval, int in_maxval, int power){
  init(in_minval, in_maxval, power, DEFAULT_PERC);
}

void Motor::init(int in_minval, int in_maxval, int power, int perc)
{
  /*
  //Timer setup
  cli();
  TCCR2A = 0;                                    // set entire TCCR1A register to 0
  TCCR2B = 0;                                    // same for TCCR1B
  TCNT2  = 0;                                    // initialize counter value to 0
  OCR2A  = 255;                                  // COMPARE REGISTER A = (16*10^6) / (1*1024) - 1 (must be <256 -> 8 bit)-------> [16*10^6/(prescale*desired frequncy)] -1
  TCCR2B |= (1 << WGM22);                        // turn on CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS20);           // Set CS12 and CS10 bits for 1024 prescaler
  TIMSK2 |= (0 << OCIE2A);                       // enable timer compare interrupt
  //*/

  this->in_minval = in_minval;
  this->in_maxval = in_maxval;

  //boundaries check
  if(power < 0) power = 0;
  else if (power >= MAX_POWER) power = MAX_POWER;

  this->maxval      = SERVO_STOP_VALUE + power;
  this->minval      = SERVO_STOP_VALUE - power;

  /*percentage check
  if (perc >= DEFAULT_MAX_PERC)
    perc = DEFAULT_MAX_PERC;
  else if (perc <= 0)
    perc = DEFAULT_PERC; 

  //boundaries and step setup
  this->step        = (this->maxval - this->minval) * perc / 100;

  //functional values setup
  this->value       = SERVO_STOP_VALUE;
  this->reach_value = this->value;
  sei(); //*/
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
  this->value = SERVO_STOP_VALUE;
  this->reach_value = this->value;
  
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
  if (this->value < this->reach_value)
   {
      if (abs(this->value - this->reach_value) < this->step)
      {
        this->value = this->reach_value;
      }
      else
      {
        this->value += this->step;
      }
   }
   else if (this->value > this->reach_value)
   {
      if (abs(this->value - this->reach_value) < this->step)
      {
        this->value = this->reach_value;
      }
      else
      {
        this->value -= this->step;
      }
   }

  this->motor.writeMicroseconds(this->value);

  return this->is_value_reached();
}


/** Method to set the reach_value
 *
 *  @param val
 *
 *  @return void
 */
void Motor::set_value(int val)                                      // set the new value of the current to reach and the step
{
  if (val > this->in_minval) val = this->in_minval;                         // saturation max value
  if (val < this->in_maxval) val = this->in_maxval;                         // stauration min value
  this->reach_value = map(val, this->in_minval, this->in_maxval, this->minval, this->maxval);

  /* DEBUG 
  Serial.print("\tMotor reach value:\t");
  Serial.println(this->reach_value);
  
  if (!this->update()){
    insert(this->code);
    TIMSK2 |= (1 << OCIE2A);
  }*/
  this->motor.writeMicroseconds(this->reach_value);
}


/** method to check if the final value has been reached **/
bool Motor::is_value_reached(){
  return this->value == this->reach_value;
}


/** constructor **/
Motor::Motor()
{
  //motor setup
  this->code        = i;
  vect[i] = this;
  i++;

  this->pin = -1;
}


/** getters **/
int Motor::get_pin(){
  return this->pin;
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
