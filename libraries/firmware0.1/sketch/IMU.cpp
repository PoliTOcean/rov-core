#include <Wire.h>
#include "Arduino.h"
#include "IMU.h"

#define MPU_ADDR      0x68
#define ACCEL_XOUT_H  0x3B
#define ACCEL_XOUT_L  0x3C
#define ACCEL_YOUT_H  0x3D
#define ACCEL_YOUT_L  0x3E
#define ACCEL_ZOUT_H  0x3F
#define ACCEL_ZOUT_L  0x40
#define TEMP_OUT_H    0x41
#define TEMP_OUT_L    0x42
#define GYRO_XOUT_H   0x43
#define GYRO_XOUT_L   0x44
#define GYRO_YOUT_H   0x45
#define GYRO_YOUT_L   0x46
#define GYRO_ZOUT_H   0x47
#define GYRO_ZOUT_L   0x48


void IMU::configure(){
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  cdt = 0;
}

void IMU::complementaryFilter(){
    float pitchAcc, rollAcc, droll, dpitch, dyaw;
    float cdr, cdp, cdy, cr, cp;
    float sdr, sdp, sdy, sr, sp;
    float accTot;

    float dt = (float) (micros() - lastUpdate)/100; //todo check it!
    lastUpdate = micros(); // update `lastUpdate` for the next iteration
    
    // add real dt with micros
    droll = -Gx * dt;   // Angle around the X-axis (upside-down)
    dpitch = -Gy * dt;  // Angle around the Y-axis (upside-down)
    dyaw = Gz * dt;    // Angle around the Z-axis

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
        rollAcc = atan2(Ay, -Az); //(upside-down, accData[2])
        pitchAcc = asin(-Ax/fabs(accTot));

        roll = roll * 0.9 + rollAcc * 0.1;
        pitch = pitch * 0.9 + pitchAcc * 0.1;   
    }

    dt = 0;
}


void IMU::imuRead(){
  
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 14, true); // request a total of 14 registers

  unsigned long now = micros();
  dt += now - lastUpdate;       
  lastUpdate = now;
  
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
  Ax = float(Ax - 1089.4)/16436;
  Ay = float(Ay + 496.4)/16357;
  Az = float(Az + 1396.8)/16802.6;

  // Convert the data
  Gx = (Gx + 159.07)/1800; //2700 @10ms
  Gy = (Gy - 115.9)/1600; //2500 @10ms
  Gz= (Gz + 141.44)/1600; //2500 @10ms
  
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
