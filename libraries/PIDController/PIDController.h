#ifndef PIDController_H_
#define PIDController_H_

class PIDController
{

private:
    float KP, KI, KD;
    float dt;
    float MAX;
    float prev;
    float integ;

public:
    PIDController(float KP, float KI, float KD, float dt)
        : PIDController(KP, KI, KD, dt, 0){}

    PIDController(float KP, float KI, float KD, float dt, float MAX)
        : PIDController(KP, KI, KD, dt, MAX, 0) {}
    
    PIDController(float KP, float KI, float KD, float dt, float MAX, float prev)
        : KP(KP), KI(KI), KD(KD), dt(dt), MAX(MAX), prev(prev), integ(0) {}

    float calculate_power(float current, float desired);
    void reset();

};





#endif //PIDController_H_