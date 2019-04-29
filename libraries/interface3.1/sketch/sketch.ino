/*
 * Created by pettinz.
 */

#include <Sensor.h>
#include <Array.h>
#include <SPI.h>
#include "IMU.h"

#define SENSORS_SIZE static_cast<int>(sensor_t::Last)+1

volatile sensor_t s; // current sensor
volatile Array<Sensor<byte>, SENSORS_SIZE> sensors;

volatile bool process;
volatile byte c;

IMU imu;

void setup() {
    Serial.begin(9600);

    imu.configure();
    
    pinMode(MISO, OUTPUT);
    SPCR |= _BV(SPE);
    SPDR = 0xFF;
    
    process = false;
    
    SPI.attachInterrupt();
    
    // create sensors array
    for (auto sensor_type : sensor_t())
        sensors.push_back(Sensor<byte>(sensor_type, 0));

    s = sensor_t::First;
}

void loop() {
  imu.printValues();
  
  sensors[static_cast<int>(sensor_t::PITCH)].setValue(imu.getAx());
  sensors[static_cast<int>(sensor_t::ROLL)].setValue(imu.getAy());
  //sensors[static_cast<int>(sensor_t::YAW)].setValue(imu.getGz());

  if (process)
  {
    Serial.println(sensors[static_cast<int>(sensor_t::PITCH)].getValue());
    process = false;
  }
}

ISR (SPI_STC_vect)
{
    c = SPDR;
    
    // Prepare the next sensor's value to send through SPI
    SPDR = sensors[static_cast<int>(s)].getValue();
  
    // if I sent the last sensor, reset current sensor to first one.
    if (++s > sensor_t::Last)
      s = sensor_t::First;

    process = true;
}
