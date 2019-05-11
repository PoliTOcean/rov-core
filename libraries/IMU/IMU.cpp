#include <Wire.h>
#include "Arduino.h"
#include "IMU.h"

void IMU::configure(){
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
}

void IMU::complementaryFilter(){
    float pitchAcc, rollAcc, droll, dpitch, dyaw;
    float cdr, cdp, cdy, cr, cp;
    float sdr, sdp, sdy, sr, sp;
    float accTot;
    
    // add real dt with micros
    droll   = -Gx * IMU_dT;   // Angle around the X-axis (upside-down)
    dpitch  = -Gy * IMU_dT;   // Angle around the Y-axis (upside-down)
    dyaw    =  Gz * IMU_dT;   // Angle around the Z-axis

    cdr=cos(droll);
    cdp=cos(dpitch);
    cdy=cos(dyaw);

    cr=cos(roll);
    cp=cos(pitch);

    sdr=sin(droll);
    sdp=sin(dpitch);
    sdy=sin(dyaw);

    sr=sin(roll);
    sp=sin(pitch);

    roll=atan2(cdr*(sdy*sp + cdy*cp*sr) - sdr*(sdp*(cdy*sp - cp*sdy*sr) - cdp*cp*cr), - sdr*(sdy*sp + cdy*cp*sr) - cdr*(sdp*(cdy*sp - cp*sdy*sr) - cdp*cp*cr));
    pitch=atan2(cdp*(cdy*sp - cp*sdy*sr) + cp*cr*sdp, sqrt(pow(cdr*(sdy*sp + cdy*cp*sr) - sdr*(sdp*(cdy*sp - cp*sdy*sr) - cdp*cp*cr),2)+pow(- sdr*(sdy*sp + cdy*cp*sr) - cdr*(sdp*(cdy*sp - cp*sdy*sr) - cdp*cp*cr),2)));
 
    accTot = sqrt(pow(abs(Ax),2) + pow(abs(Ay),2) + pow(abs(Az),2));
    if (accTot > 0.9 && accTot < 1.1)
    {
        rollAcc = atan2(Ay, Az);
        pitchAcc = asin(-Ax/fabs(accTot));

        roll = roll * 0.9 + rollAcc * 0.1 - 0.1;
        pitch = pitch * 0.9 + pitchAcc * 0.1;   
    }

    float tmp;
    tmp       = pitch;
    pitch     = roll;
    roll      = tmp;
}


void IMU::imuRead(){
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // request a total of 14 registers
  
  Ax = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  Ay = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  Az = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  Gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  Gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  Gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //scale temperature
  Tmp = (Tmp/340.00+36.53);

  // Convert the data
  Ax = int(Ax - 328.063)/16895.24;
  Ay = int(Ay - 335.44)/16604.259;
  Az = -int(Az - 1490.462)/16954.01;

  // Convert the data
  Gx = (Gx - 349.262)/3300;
  Gy = (Gy - 154.551)/3350;
  Gz = (Gz + 176.136)/3200;
  
  complementaryFilter(); //call calculation function
}

void IMU::printValues(){

  imuRead();
  
  Serial.print("Acc:\t");
  Serial.print("X= "); Serial.print(Ax);
  Serial.print("\tY= "); Serial.print(Ay);
  Serial.print("\tZ= "); Serial.println(Az);
  //equation for temperature in degrees C from datasheet
  Serial.print("Temperature: "); Serial.print(Tmp); Serial.println(" °C ");

  Serial.print("Gyr:\t");
  Serial.print("X= "); Serial.print(Gx);
  Serial.print("\tY= "); Serial.print(Gy);
  Serial.print("\tZ= "); Serial.println(Gz);
  Serial.println("****");
}

/*byte IMU::getAx(){
  updateValues();   
  return ((Ax/256) & 0x00FF);
}

byte IMU::getAy(){
  updateValues();
  return ((Ay/256) & 0x00FF);
}

byte IMU::getAz(){
  updateValues();
  return ((Az/256) & 0x00FF);
}


byte IMU::getGx(){
  updateValues();
  return ((Gx/256) & 0x00FF);
}

byte IMU::getGy(){
  updateValues();
  return ((Gy/256) & 0x00FF);  
}

byte IMU::getGz(){
  updateValues();
  return ((Gz/256) & 0x00FF);  
}*/
