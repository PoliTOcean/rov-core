#include "PIDController.h"
#define abs(a) (a<0?-a:a)

float PIDController::calculate_power(float current){
    float der, power_p, power_d, power_i, power=0;

    if ( abs(current) > threshold )
    {        
        power_p   = KP*current;

        der       = (current-prev)/dt;
        power_d   = KD*der;

        integ     = integ + dt*current;
        power_i   = KI*integ;

        power     = power_p + power_d + power_i;
    }
    else
      integ   = 0;

    prev = current;

    return ( MAX > 0 && abs(power) > MAX ) ? MAX : power;
}