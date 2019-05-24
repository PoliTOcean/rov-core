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
    droll   = -Gx * dt;   // Angle around the X-axis (upside-down)
    dpitch  = -Gy * dt;   // Angle around the Y-axis (upside-down)
    dyaw    =  Gz * dt;   // Angle around the Z-axis

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
    pitch=atan2(cdp*(cdy*sp - cp*sdy*sr) + cp*cr*sdp, sqrt( abs( pow(cdr*(sdy*sp + cdy*cp*sr) - sdr*(sdp*(cdy*sp - cp*sdy*sr) - cdp*cp*cr),2)+pow(- sdr*(sdy*sp + cdy*cp*sr) - cdr*(sdp*(cdy*sp - cp*sdy*sr) - cdp*cp*cr),2) ) ) );
 
    accTot = sqrt( abs( pow(Ax,2) + pow(Ay,2) + pow(Az,2) ) );
    if (accTot > 0.9 && accTot < 1.1)
    {
        rollAcc = atan2(Ay, Az);
        pitchAcc = asin(-Ax / accTot);

        roll = roll * 0.8 + rollAcc * 0.2;
        pitch = pitch * 0.8 + pitchAcc * 0.2;   
    }
}


void IMU::imuRead(){
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // request a total of 14 registers
  
  // Accelerometer values
  Ax = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  Ay = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  Az = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  // Temperature
  temperature   = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  // Gyroscope values
  Gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  Gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  Gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

  //scale temperature
  temperature = (temperature/340.00+36.53);

  // Convert the data
  Ax = -(Ax - 337.9)   / 16726.05;
  Ay = -(Ay - 415.107) / 16189.153;
  Az = (Az - 1660.462)  / 17123.54;

  // Convert the data
  Gx = -(Gx - 349.262) / 2500;
  Gy = -(Gy - 154.551) / 2500;
  Gz = (Gz + 176.136) / 2500;
  
  complementaryFilter(); //call calculation function
}

void IMU::printValues(){

  imuRead();
  
  Serial.print("Acc:\t");
  Serial.print("X= "); Serial.print(Ax);
  Serial.print("\tY= "); Serial.print(Ay);
  Serial.print("\tZ= "); Serial.print(Az);
  //equation for temperature in degrees C from datasheet
  Serial.print("\tTemperature: "); Serial.print(temperature); Serial.print(" °C ");

  Serial.print("\tGyr:\t");
  Serial.print("X= "); Serial.print(Gx);
  Serial.print("\tY= "); Serial.print(Gy);
  Serial.print("\tZ= "); Serial.println(Gz);
}