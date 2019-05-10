#include "Arduino.h"
#include "Motors.h"

#define IN_AXES_MIN 1
#define IN_AXES_MAX 254
#define AXES_MAX    127
#define AXES_MIN    -126

#define UR_pin  8
#define UL_pin  7
#define UB_pin  4
#define FR_pin  5
#define FL_pin  6
#define BR_pin  3
#define BL_pin  2

#define kAng  15
#define V_MUL 120
#define kDep  600


void Motors::configure(MS5837 *psensor, IMU imu){
    // attach motors
    UR.attach(UR_pin);
    UL.attach(UL_pin);
    UB.attach(UB_pin);
    FR.attach(FR_pin);
    FL.attach(FL_pin);
    BR.attach(BR_pin);
    BL.attach(BL_pin);

    Motors::stop();  // do not run the motors untill `start()` is called
    brSensor = psensor; // catch the pressure sensor object
    imuSensor = imu; // catch the imu sensor object
    savePressure = false;
    powerMode = MEDIUM;

    configured = true;
}

//function for pitch power calculation
float Motors::calcPitchPower(){
  /* it takes the difference between current pitch and the requested one from the joystick
   * and multiplicates it for a multiplication constant, passed as parameter */
  return 0;//kAng*imuSensor.getPitch(); //(the angle is the orizontal due to the sensor inclination)
  // TO DO controllare per MAX IMU
}

//function for roll power calculation. Same as above, without sign inversion
float Motors::calcRollPower(){
  return 0;//kAng*imuSensor.getRoll(); //(the angle is the orizontal due to the sensor inclination)
  // TO DO controllare per MAX IMU
}

//function to evaluate vertical motors values
void Motors::evaluateVertical(){
   if(!configured) return;

   float pitchPower, rollPower;
   if(!started){
     UR.set_value(0);
     UL.set_value(0);
     UB.set_value(0);
     return;
   }

   //call above functions for calculations
   pitchPower = calcPitchPower();
   rollPower  = calcRollPower();
   
   //value for up-down movement
   int valUD=0, autoQuote=0;            //reset valUD
   if(down>0 || up>0){     //controlled up-down from joystick
     savePressure = true;                           //it has to save pressure when finished
     valUD = (up-down)*V_MUL; //fixed value depending on buttons pressed
   }else if(savePressure){
     requested_pressure = current_pressure;
     savePressure = false;
   } //else, if it is not (still) pressing up/down buttons
   
   if(!savePressure) //change value for autoquote
     autoQuote = -(requested_pressure-current_pressure)*kDep;

    Serial.print(requested_pressure);
    Serial.print("\t");
    Serial.print(current_pressure);
    Serial.print("\t");
    Serial.println(autoQuote);
  
   //adding values for UD movement/autoquote
   UL.set_value(valUD + ( autoQuote - pitchPower - rollPower) / mulPower[powerMode] );
   UR.set_value(valUD + ( autoQuote - pitchPower + rollPower) / mulPower[powerMode] );
   UB.set_value(valUD + ( autoQuote + 2*pitchPower) / mulPower[powerMode] );
}

/* function to evaluate powers for horizontal movement.*/
// TODO we have to pass from the SPI x y and rz and set the motors values
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

void Motors::setCurrentPressure(float currPress){
  current_pressure = currPress;
}

void Motors::start(){
  
  /* DEBUG */
  Serial.println("Starting...");
  
  started = true;
}

void Motors::stop(){
  
  /* DEBUG */
  Serial.println("Stopping...");
  
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