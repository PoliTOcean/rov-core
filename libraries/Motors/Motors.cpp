#include "Arduino.h"
#include "Motors.h"

#define PWR_CUT_PERC  0.65            // 65%
#define PWR_THRESHOLD MAX_POWER*7*0.9 // 90% of total power

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
    setPower(power::SLOW);

    configured = true;
}

//function to evaluate vertical motors values
void Motors::evaluateVertical(int current_pressure, float roll, float pitch){
   if(!configured) return;

   float pitchPower, rollPower;
   if(!started){
     UR.stop();
     UL.stop();
     UB.stop();
     return;
   }

   //call above functions for calculations
   pitchPower = pitchCorrection.calculate_power(pitch, 0);
   rollPower  = rollCorrection.calculate_power(roll, 0);
   
   //value for up-down movement
   int valUD=0, depthCorrectionPower=0;            //reset valUD
   if(down>0 || up>0){     //controlled up-down from joystick
     savePressure = true;                           //it has to save pressure when finished
     valUD = (up-down)*axis_max; //fixed value depending on buttons pressed
   }else if(savePressure){
     requested_pressure = current_pressure;
     savePressure = false;
   } //else, if it is not (still) pressing up/down buttons
   else //change value for autoquote
     depthCorrectionPower = depthCorrection.calculate_power(current_pressure, requested_pressure);
/* DEBUG
   Serial.print("Pitch: ");
   Serial.print(pitch);
   Serial.print("\tRoll: ");
   Serial.print(roll);
   Serial.print("\tPitch power: ");
   Serial.print(pitchPower);
   Serial.print("\tRoll power: ");
   Serial.print(rollPower);
   Serial.print("\tRequested pressure: ");
   Serial.print(requested_pressure);
   Serial.print("\tCurrent pressure: ");
   Serial.print(current_pressure);
   Serial.print("\tDepth correction power: ");
   Serial.println(depthCorrectionPower);
*/
   //adding values for UD movement/autoquote
   UL.set_offset( depthCorrectionPower + pitchPower + rollPower );
   UR.set_offset( depthCorrectionPower + pitchPower - rollPower );
   UB.set_offset( depthCorrectionPower + 2*pitchPower );
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

/* function to evaluate powers for horizontal movement.*/
void Motors::evaluateHorizontal() {
  if(!started){
    FL.stop();
    FR.stop();
    BL.stop();
    BR.stop();
    return;
  }

  int rz = this->rz * 0.7;
  FL.set_value(signFL * (-y+x+rz));
  FR.set_value(signFR * (-y-x-rz));
  BL.set_value(signBL * (-y-x+rz));
  BR.set_value(signBR * (-y+x-rz));

  // if the rov request full power  for all its motors reduce the
  // value of BL and BR in a certain percentage in order to 
  // prevent ESC protection

  if(powerMode == power::FAST
    && getTotalPower() > PWR_THRESHOLD){
      int new_BL = BL.get_value() * PWR_CUT_PERC;
      int new_BR = BR.get_value() * PWR_CUT_PERC;
      BL.set_value(new_BL);
      BR.set_value(new_BR);
  }
}

void Motors::start(int current_pressure){  
  started = true;

  requested_pressure = current_pressure;
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
  total += FR.get_value();
  total += FL.get_value();
  total += BR.get_value();
  total += BL.get_value();
  total += UR.get_value();
  total += UL.get_value();
  total += UB.get_value();
  return total;
}
