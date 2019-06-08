#include "Arduino.h"
#include "Motors.h"

#define PWR_CUT_PERC  0.4             // 40%
#define PWR_THRESHOLD MAX_POWER*7*0.75 // 90% of total power

#define N_PRESS_SAMPLES       5
#define N_SECS                3
#define PRESS_TIME_THRESHOLD  N_SECS*1000000/N_PRESS_SAMPLES //us

#define UR_pin  8
#define UL_pin  7
#define UB_pin  4
#define FR_pin  5
#define FL_pin  6
#define BR_pin  3
#define BL_pin  2


void Motors::configure(){
    // attach motors
    UR.attach(UR_pin);
    UL.attach(UL_pin);
    UB.attach(UB_pin);
    FR.attach(FR_pin);
    FL.attach(FL_pin);
    BR.attach(BR_pin);
    BL.attach(BL_pin);

    stop();                 // do not run the motors untill `start()` is called
    savePressure = false;
    pitchControlOff();
    setPower(power::SLOW);

    configured = true;
}

//function to evaluate vertical motors values
void Motors::evaluateVertical(float current_pressure, float roll, float pitch){
   float pitchPower, rollPower;
   if(!started){
     UR.stop();
     UL.stop();
     UB.stop();
     return;
   }

   
   //value for up-down movement
   int valUD=0;
   float depthCorrectionPower=0;
   if(down>0 || up>0){            //controlled up-down from joystick
     savePressure = true;         //it has to save pressure when finished
     valUD = (up-down)*axis_max;  //fixed value depending on buttons pressed
   }
   else
   {
      if (savePressure)
      {
        pressCount = 0;
        savePressure = false;
      }
      else if (pressCount < N_PRESS_SAMPLES && micros()-startTimeForPressure > (long) PRESS_TIME_THRESHOLD)
      {
        requested_pressure = current_pressure;
        startTimeForPressure = micros();
        depthCorrection.reset();
        pressCount++;
      }
      depthCorrectionPower = -depthCorrection.calculate_power(current_pressure, requested_pressure);
   }

   if (pitchControlEnabled) {
    pitchPower = pitchControlPower;
    depthCorrectionPower = 0;
    rollPower = 0;
    depthCorrection.reset();
    pitchCorrection.reset();
   } else {
    pitchPower = pitchCorrection.calculate_power(pitch, 0.02);
    rollPower  = rollCorrection.calculate_power(roll, 0);
   } 
   
   //adding values for UD movement/autoquote
   UL.set_offset( depthCorrectionPower + pitchPower + rollPower );
   UR.set_offset( depthCorrectionPower + pitchPower - rollPower );
   UB.set_offset( depthCorrectionPower - 2*pitchPower );
   UL.set_value(valUD, true);
   UR.set_value(valUD, true);
   UB.set_value(valUD, true);
}

void Motors::writeMotors(){
  UL.write();
  UR.write();
  UB.write();
  FL.write();
  FR.write();
  BL.write();
  BR.write();
}

void Motors::evaluateHorizontal() {
  if(!started){
    FL.stop();
    FR.stop();
    BL.stop();
    BR.stop();
    return;
  }

  //rotation inibition
  int rz = this->rz;
  switch(powerMode)
  {
    case SLOW:    rz = 0.8*rz; break;
    case MEDIUM:  rz = 0.6*rz; break;
    case FAST:    rz = 0.4*rz; break;
  }
  
  // should we update?
  bool updateNow = timer.onRestart();

  // set values
  FL.set_value( signFL * (-y + x + rz), updateNow);
  FR.set_value( signFR * (-y - x - rz), updateNow);
  int valueBL = signBL * (-y - x + rz);
  int valueBR = signBR * (-y + x - rz);
  BL.set_value(valueBL, updateNow);
  BR.set_value(valueBR, updateNow);

  // if the rov request full power  for all its motors reduce the
  // value of BL and BR in a certain percentage in order to 
  // prevent 12V power protection
  if(powerMode == power::FAST
    && getTotalPower() > PWR_THRESHOLD){
      BL.set_value(valueBL * PWR_CUT_PERC, true);
      BR.set_value(valueBR * PWR_CUT_PERC, true);
  }
}

void Motors::start(){
  if(!configured) return;
  started = true;
  savePressure = true;
}

void Motors::stop(){  
  x = 0;
  y = 0;
  rz = 0;
  started = false;
}

void Motors::stopUp(){
  up = 0;
}

void Motors::stopDown(){
  down = 0;
}

void Motors::goUp(){
  up = 0.6;
}

void Motors::goUpFast(){
  up = 1;
}

void Motors::stopUpFast(){
  up = 0;
}

void Motors::goDown(){
  down = 0.8;
}

void Motors::setX(int x){
  if (x < axis_min) x = axis_min;
  else if (x > axis_max) x = axis_max;
  this->x = x;
}

void Motors::setY(int y){
  if (y < axis_min) y = axis_min;
  else if (y > axis_max) y = axis_max;
  this->y = y;
}

void Motors::setRz(int rz){
  if (rz < axis_min) rz = axis_min;
  else if (rz > axis_max) rz = axis_max;
  this->rz = rz;
}

void Motors::setPower(power pwr){
  int perc = horizontalPowerPerc[static_cast<int>(pwr)];
  FR.set_power(perc);
  FL.set_power(perc);
  BR.set_power(perc);
  BL.set_power(perc);

  perc = verticalPowerPerc[static_cast<int>(pwr)];
  UR.set_power(perc);
  UL.set_power(perc);
  UB.set_power(perc);

  this->powerMode = pwr;
}

int Motors::getTotalPower(){
  int total = 0;
  total += FR.get_value()-SERVO_STOP_VALUE;
  total += FL.get_value()-SERVO_STOP_VALUE;
  total += BR.get_value()-SERVO_STOP_VALUE;
  total += BL.get_value()-SERVO_STOP_VALUE;
  total += UR.get_value()-SERVO_STOP_VALUE;
  total += UL.get_value()-SERVO_STOP_VALUE;
  total += UB.get_value()-SERVO_STOP_VALUE;
  return total;
}

void Motors::setPitchControlPower(int pitchPower) {
  this->pitchControlPower = pitchPower;
}

void Motors::pitchControlOn() {
  this->pitchControlEnabled = true;
}

void Motors::pitchControlOff() {
  this->pitchControlEnabled = false;
}