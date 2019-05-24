#include "PIDController.h"
#define abs(a) (a<0?-a:a)

float PIDController::calculate_power(float currentValue, float desired){
    float der, power_p, power_d, power_i, power=0;
    
    float current = desired - currentValue; //current error
   
    power_p   = KP*current;

    der       = (current-prev)/dt;
    power_d   = KD*der;

    integ     = integ + dt*current;
    power_i   = KI*integ;

    power     = power_p + power_d + power_i;

    prev = current;

    if(MAX > 0)
      if(power > MAX) power = MAX;
      else if(power < -MAX) power = -MAX;

    return power;
}