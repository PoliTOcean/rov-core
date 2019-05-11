#include "Arduino.h"
#include "Motors.h"

#define IN_AXES_MIN 1
#define IN_AXES_MAX 254
#define AXES_MAX    127
#define AXES_MIN    -126

#define MAX_IMU     80

#define UR_pin  8
#define UL_pin  7
#define UB_pin  4
#define FR_pin  5
#define FL_pin  6
#define BR_pin  3
#define BL_pin  2

#define kAng  80
#define kDep  35


void Motors::configure(){
    // attach motors
    UR.attach(UR_pin);
    UL.attach(UL_pin);
    UB.attach(UB_pin);
    FR.attach(FR_pin);
    FL.attach(FL_pin);
    BR.attach(BR_pin);
    BL.attach(BL_pin);

    Motors::stop();       // do not run the motors untill `start()` is called
    savePressure = false;
    powerMode = MEDIUM;

    configured = true;
}

//function for pitch power calculation
float Motors::calcPitchPower(float pitch){
  int power = kAng*pitch; //(the angle is the orizontal due to the sensor inclination)
  if(power > MAX_IMU) power = MAX_IMU;
  return power;
}

//function for roll power calculation. Same as above, without sign inversion
float Motors::calcRollPower(float roll){
  int power = kAng*roll; //(the angle is the orizontal due to the sensor inclination)
  if(power > MAX_IMU) power = MAX_IMU;
  return power;
}

//function to evaluate vertical motors values
void Motors::evaluateVertical(float current_pressure, float roll, float pitch){
   if(!configured) return;

   float pitchPower, rollPower;
   if(!started){
     UR.set_value(0);
     UL.set_value(0);
     UB.set_value(0);
     return;
   }

   //call above functions for calculations
   pitchPower = calcPitchPower(pitch);
   rollPower  = calcRollPower(roll);
   
   //value for up-down movement
   int valUD=0, depthControl=0;            //reset valUD
   if(down>0 || up>0){     //controlled up-down from joystick
     savePressure = true;                           //it has to save pressure when finished
     valUD = (up-down)*AXES_MAX; //fixed value depending on buttons pressed
   }else if(savePressure){
     requested_pressure = current_pressure;
     savePressure = false;
   } //else, if it is not (still) pressing up/down buttons
   else //change value for autoquote
     depthControl = -(requested_pressure-current_pressure)*kDep;

   //adding values for UD movement/autoquote
   UL.set_value(valUD + ( depthControl - pitchPower - rollPower) / mulPower[powerMode] );
   UR.set_value(valUD + ( depthControl - pitchPower + rollPower) / mulPower[powerMode] );
   UB.set_value(valUD + ( depthControl + 2*pitchPower) / mulPower[powerMode] );
}

/* function to evaluate powers for horizontal movement.*/
void Motors::evaluateHorizontal() {
  if(!started){
    FL.set_value(0);
    FR.set_value(0);
    BL.set_value(0);
    BR.set_value(0);
    return;
  }
  // I puntatori si riferiscono ai motori
  FL.set_value(signFL * (-y+x+rz));
  FR.set_value(signFR * (-y-x-rz));
  BL.set_value(signBL * (-y-x+rz));
  BR.set_value(signBR * (-y+x-rz));
}

void Motors::start(float current_pressure){  
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
  up = 1;
}

void Motors::goUpFast(){
  up = 1.5;
}

void Motors::stopUpFast(){
  up = 1;
}

void Motors::goDown(){
  down = 1;
}

void Motors::setX(byte x){
  this->x = x-127;
}

void Motors::setY(byte y){
  this->y = y-127;
}

void Motors::setRz(byte rz){
  this->rz = rz-127;
}

void Motors::setPower(power pwr){
  if(!configured) return;

  float mul = mulPower[static_cast<int>(pwr)];

  FR.setPower((int)(mul*DEFAULT_POWER));
  FL.setPower((int)(mul*DEFAULT_POWER));
  BR.setPower((int)(mul*DEFAULT_POWER));
  BL.setPower((int)(mul*DEFAULT_POWER));

  if(pwr == SLOW) mul = ( mulPower[static_cast<int>(MEDIUM)]+mulPower[static_cast<int>(SLOW)] ) / 2;
  UR.setPower((int)(mul*DEFAULT_POWER));
  UL.setPower((int)(mul*DEFAULT_POWER));
  UB.setPower((int)(mul*DEFAULT_POWER));

  this->powerMode = pwr;
}