#include "Arduino.h"
#include "Motors.h"

#define M0_pin  4
#define M1_pin  5
#define M2_pin  6
#define M3_pin  11
#define M4_pin  12
#define M5_pin  13
#define M6_pin  14



void Motors::configure(PressureSensor psensor, IMU imu){
    cli();                                        //stop interrupts
      // attach motors
      M0.attach(M0_pin);                              
      M1.attach(M1_pin);                              
      M2.attach(M2_pin);                              
      M3.attach(M3_pin);                              
      M4.attach(M4_pin);                             
      M5.attach(M5_pin);                              
      M6.attach(M6_pin);                              
    sei();

    pressureSensor = psensor; // catch the pressure sensor object
    imuSensor = imu; // catch the imu sensor object
}
void Motors::control(){
//    if(start){                            //if the ROV is started
//      #ifdef TEST                                     //if we are in water
//          evaluateVertical(K_ANG, K_DEP, vertical);   //then evaluate vertical values for autostabilization and autoquote
//      #else                                           //else, if we aren't,
//          evaluateVertical(0, 0, vertical);           //then evaluate them just for joystick up/down
//      #endif
//          
//          evaluateHorizontal(&valLF, &valRF, &valLB, &valRB);           //evaluate values for horizontal movement
//
//    //set new motors powers
//    //setServosValues(valLF, valRF, valLB, valRB, vertical[0], vertical[1], vertical[2], vertical[3], MAX_SRV);
//  }else                                 //else, if the ROV is stopped,
//    //setServosValues(0, 0, 0, 0, 0, 0, 0, 0, 0);       //then send STOP signal to motors
}

//function for pitch power calculation
float Motors::calcPitchPower(float kAng){
  /* it takes the difference between current pitch and the requested one from the joystick
   * and multiplicates it for a multiplication constant, passed as parameter */
  return kAng*(imuSensor.pitch+(15.0*3.14/180)); //(the angle is the orizontal due to the sensor inclination)
}

//function for roll power calculation. Same as above, without sign inversion
float Motors::calcRollPower(float kAng){
  return kAng*(imuSensor.roll+(5.0*3.14/180)); //(the angle is the orizontal due to the sensor inclination)
}

//function to evaluate vertical motors values
void Motors::evaluateVertical(float kAng, float kDep, int vertical[4]){
//   if(trigger)
//   {
//    up = 1;
//    down = 0;
//    fastV = 0;
//   }
//   if(trigger2)
//   {
//    up = 1;
//    down = 0;
//    fastV = 1;
//   }
//   else if(pinkie)
//   {
//    up = 0;
//    down = 1;
//    fastV = 0;
//   }
//   else if(!trigger && !trigger2 && !pinkie)
//   {
//    up = 0;
//    down = 0;
//    fastV = 0;
//   }
//   
//    float pitchPower, rollPower;
//    //call above functions for calculations
//    pitchPower = calcPitchPower(kAng);
//    rollPower = calcRollPower(kAng);
//  
//    //set values for pitch
//    vertical[0] =  -linSaturation(pitchPower, MAX_IMU);
//    vertical[1] =  -linSaturation(pitchPower, MAX_IMU);
//    vertical[2] = linSaturation(pitchPower, MAX_IMU);
//    vertical[3] = linSaturation(pitchPower, MAX_IMU);
//  
//    //adding values for roll
//    vertical[0] -= linSaturation(rollPower, MAX_IMU);
//    vertical[1] += linSaturation(rollPower, MAX_IMU);
//    vertical[2] -= linSaturation(rollPower, MAX_IMU);
//    vertical[3] += linSaturation(rollPower, MAX_IMU);
//    
//    //value for up-down movement
//    valUD=0;            //reset valUD
//    if(down || up){         //controlled up-down from joystick
//      savePressure=1;                           //it has to save pressure when finished
//      valUD = -(up-down)*(V_MUL+fastV*FAST_V);  //fixed value depending on buttons pressed
//    }else if(savePressure) //else, if it is not (still) pressing up/down buttons
//      saveRequestedPressure(); //save pressure for autoquote
//    
//    if(!savePressure) //change value for autoquote
//      valUD = (reqPress-curPress)*kDep;
//  
//    //adding values for UD movement/autoquote
//    vertical[0] += linSaturation(valUD, MAX_UD);
//    vertical[1] += linSaturation(valUD, MAX_UD);
//    vertical[2] += linSaturation(valUD, MAX_UD);
//    vertical[3] += linSaturation(valUD, MAX_UD);
}

/* function to evaluate powers for horizontal movement.*/
// TODO we have to pass from the SPI x y and rz and set the motors values
void evaluateHorizontal(int *leftFront,int  *rightFront,int  *leftBack,int  *rightBack) {
 // { // I puntatori si riferiscono ai motori
//    int signLF = -1; int signRF = 1; int signLB = -1; int signRB = 1;
//    *leftFront = H_MUL * signLF * (-y+x+rz);
//    *rightFront = H_MUL * signRF* (-y-x-rz);
//    *leftBack = H_MUL * signLB * (-y-x+rz);
//    *rightBack = H_MUL * signRB * (-y+x-rz);
//  }
}
