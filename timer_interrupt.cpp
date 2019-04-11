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
      Serial.println("stop timer");
      TIMSK1 &= (0 << OCIE1A);
  }
}

void StampaLista(list_Motor* l) {
  if(l == NULL)
    {
      Serial.println(" ");
      return;
    }

  Serial.print(l->p), Serial.print(" ");
  StampaLista(l->s);
}

void Motor::init(int maxi =  DEFAULT_MAX_VAL, int mini = DEFAULT_MIN_VAL, int perc = DEFAULT_PERC)
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
 }
this->stamp();
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


void Motor::stamp()
{
  int perc;
  Serial.print("list :"),           StampaLista(list);
  Serial.print("actual value: "),   Serial.println(this->value);
  Serial.print("maxval: "),         Serial.println(this->maxval);
  Serial.print("minval: "),         Serial.println(this->minval);
  Serial.print("step: "),           Serial.println(this->step);
  Serial.print("reach value: "),    Serial.println(this->reach_value);
  perc = this->step * 100 / (DEFAULT_MAX_VAL - DEFAULT_MIN_VAL);
  Serial.print("perc: "),           Serial.println(perc);
  Serial.print("code: "),           Serial.println(this->code);
  Serial.println("-------------------------");
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

ISR(TIMER1_COMPA_vect)
{
  check_list();
}
