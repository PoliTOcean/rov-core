#include "DumpedCurrentMotor.h"
#include "Servo.h"
#include "Arduino.h"

#define MAX_VAL 127
#define MIN_VAL -127

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
 *  @param percentage of step
 *
 *  @return void
 */
void Motor::init(int maxi =  DEFAULT_MAX_VAL, int mini = DEFAULT_MIN_VAL, int perc = DEFAULT_PERC)
{
  //Timer setup
  cli();
  TCCR2A = 0;                                    // set entire TCCR1A register to 0
  TCCR2B = 0;                                    // same for TCCR1B
  TCNT2  = 0;                                    // initialize counter value to 0
  OCR2A  = 255;                                  // COMPARE REGISTER A = (16*10^6) / (1*1024) - 1 (must be <256 -> 8 bit)-------> [16*10^6/(prescale*desired frequncy)] -1
  TCCR2B |= (1 << WGM22);                        // turn on CTC mode
  TCCR2B |= (1 << CS22) | (1 << CS20);           // Set CS12 and CS10 bits for 1024 prescaler
  TIMSK2 |= (0 << OCIE2A);                       // enable timer compare interrupt

  //boundaries check
  if (maxi >= DEFAULT_MAX_VAL) maxi = DEFAULT_MAX_VAL;
  if (mini <= DEFAULT_MIN_VAL) mini = DEFAULT_MIN_VAL;

  //percentage check
  if (perc >= DEFAULT_MAX_PERC)
    perc = DEFAULT_MAX_PERC;
  else if (perc <= 0)
    perc = DEFAULT_PERC;

  //boundaries and step setup
  this->maxval      = maxi;
  this->minval      = mini;
  this->step        = (this->maxval - this->minval) * perc / 100;

  //functional values setup
  this->value       = SERVO_STOP_VALUE;
  this->reach_value = this->value;
  this->pin = -1;

  //final motor setup
  this->code        = i;
  vect[i] = this;
  i++;
  sei();
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
  if (val > MAX_VAL) val = MAX_VAL;                         // saturation max value
  if (val < MIN_VAL) val = MIN_VAL;                         // stauration min value
  this->reach_value = map(val, MIN_VAL, MAX_VAL, this->minval, this->maxval);

  /* DEBUG */
  Serial.print("\tMotor reach value:\t");
  Serial.println(this->reach_value);
  
  if (!this->update()){
    insert(this->code);
    TIMSK2 |= (1 << OCIE2A);
  }
}


/** method to check if the final value has been reached **/
bool Motor::is_value_reached(){
  return this->value == this->reach_value;
}


/** constructors **/
Motor::Motor()
{
  init();
}

Motor::Motor(int perc)
{
  init(DEFAULT_MAX_VAL, DEFAULT_MIN_VAL, perc);
}

Motor::Motor(int maxval, int minval)
{
  init (maxval, minval);
}


Motor::Motor(int maxval, int minval, int perc)
{
  init(maxval, minval, perc);
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
