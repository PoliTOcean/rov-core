#include "DumpedCurrentMotor_test2_ino.h"
#include "Arduino.h"

/*
 * in the vect vector are saved all the instances of the motor that will be create in the following way
 * Motor code    |   0    |   1    |   2    |   3    |   4    |   5    |   6   |
 * index vector  |   0    |   1    |   2    |   3    |   4    |   5    |   6   |
 */
Motor* vect[7]= {NULL, NULL, NULL, NULL, NULL, NULL, NULL};
int i = 0;
int t = 0;

struct list_Motor
{
  int  p;
  list_Motor* s;
};
list_Motor* list = NULL;

void check_list()
{
  if (list != NULL)
  {
    list_Motor* app;
    app         = list;
    if (!vect[list->p]->update()) vect[list->p]->insert();
    list        = app->s;
    delete(app);
  }
  else
  {
      TIMSK1 &= (0 << OCIE1A);
  }
}

void Motor::init(int pin, int maxi =  DEFAULT_MAX_VAL, int mini = DEFAULT_MIN_VAL, int perc = DEFAULT_PERC)
{
  this->maxval      = maxi;
  if (maxi >= DEFAULT_MAX_VAL) this->maxval = DEFAULT_MAX_VAL;
  this->minval      = mini;
  if (mini <= DEFAULT_MIN_VAL) this->minval = DEFAULT_MIN_VAL;
  this->value       = (this->maxval + this->minval) / 2;
  if (perc >= DEFAULT_MAX_PERC)
    this->step      = (this->maxval - this->minval) * DEFAULT_MAX_PERC / 100;
  else if (perc <= 0)
    this->step      = (this->maxval - this->minval) * DEFAULT_PERC / 100;
  else
    this->step      = (this->maxval - this->minval) * perc / 100;
  this->reach_value = (this->maxval + this->minval) / 2;
  this->code        = i;
  vect[i] = this;
  i++;

  this->motor.attach(pin);
}

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

    this->motor.writeMicroseconds(this->value);

 }
return this->is_value_reached();
}

void Motor::insert()
{
  list_Motor* z;
  if(list == NULL)
  {
    list = new(list_Motor);
    list->p = this->code;
    list->s = NULL;
  }
  else
  {
    z = list;
    while(z->s != NULL) z = z->s;
    z->s = new(list_Motor);
    z = z->s;
    z->p = this->code;
    z->s = NULL;
  }
}


void Motor::set_value(int val)                                      // set the new value of the current to reach and the step
{
if (val > this->maxval) val = this->maxval;                         // saturation max value
if (val < this->minval) val = this->minval;                         // stauration min value
this->reach_value = val;
if (!this->update())  this->insert();
TIMSK1 |= (1 << OCIE1A);
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

bool Motor::is_value_reached(){
  return this->value == this->reach_value;
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

Motor::Motor(int pin)
{
  init(pin);
}

Motor::Motor(int perc, int pin)
{
  init(pin, DEFAULT_MAX_VAL, DEFAULT_MIN_VAL, perc);
}

Motor::Motor(int maxval, int minval, int pin)
{
  init (pin, maxval, minval);
}


Motor::Motor(int maxval, int minval, int perc, int pin)
{
  init(pin, maxval, minval, perc);
}

ISR(TIMER1_COMPA_vect)
{
  check_list();
}
