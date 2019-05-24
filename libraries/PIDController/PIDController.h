#ifndef PIDController_H_
#define PIDController_H_

class PIDController
{

private:
    float KP, KI, KD;
    float dt;
    float threshold;
    float MAX;
    float prev;
    float integ;

public:
    PIDController(float KP, float KI, float KD, float dt)
        : PIDController(KP, KI, KD, dt, 0){}

    PIDController(float KP, float KI, float KD, float dt, float threshold)
        : PIDController(KP, KI, KD, dt, threshold, -1) {}

    PIDController(float KP, float KI, float KD, float dt, float threshold, float MAX)
        : PIDController(KP, KI, KD, dt, threshold, MAX, 0) {}
    
    PIDController(float KP, float KI, float KD, float dt, float threshold, float MAX, float prev)
        : KP(KP), KI(KI), KD(KD), dt(dt), threshold(threshold), MAX(MAX), prev(prev), integ(0) {}

    float calculate_power(float current);

};





#endif //PIDController_H_