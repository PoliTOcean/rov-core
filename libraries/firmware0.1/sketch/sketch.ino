
#include <Sensor.h>
#include <Array.h>
#include <SPI.h>
#include "IMU.h"
#include "PressureSensor.h"
#include "Motors.h"

#define SENSORS_SIZE static_cast<int>(sensor_t::Last)+1

volatile sensor_t s; // current sensor
volatile Array<Sensor<byte>, SENSORS_SIZE> sensors;

volatile bool process;
volatile byte c;
volatile float y, x, rz;

IMU imu;
PressureSensor pressure;
Motors motors;


void setup() {
    // initialize comunication system
    Serial.begin(9600);

    // initialize IMU sensor
    imu.configure();

    // initialize pressure sensor
    pressure.configure();
    
    // initialize motors
    motors.configure(pressure,imu);
    
    // delay of 1 second to make actions complete
    delay(1000);

    // SPI setup
    pinMode(MISO, OUTPUT);
    SPCR |= _BV(SPE);
    SPDR = 0xFF;
    SPI.attachInterrupt();
    
    // create sensors array
    for (auto sensor_type : sensor_t())
        sensors.push_back(Sensor<byte>(sensor_type, 0));

    s = sensor_t::First;
}

void loop() {
  // prepare data to send back via spi
  //TODO set all sensors
  sensors[static_cast<int>(sensor_t::PITCH)].setValue(imu.pitch);
  sensors[static_cast<int>(sensor_t::ROLL)].setValue(imu.roll);
}

ISR (SPI_STC_vect)
{
    c = SPDR;
    
    // Prepare the next sensor's value to send through SPI
    SPDR = sensors[static_cast<int>(s)].getValue();

    switch(s){
       case sensor_t::ROLL:         // when we send roll we read x
       motors.x = float(c-127)/127;
       break;
      
       case sensor_t::PITCH:        // when we send pitch we read y
       motors.y = float(c-127)/127;
       break;
      
       case sensor_t::TEMPERATURE:  // when we send temperature we read rz
       motors.rz = float(c-127)/127;
       break;

       // TODO change PRESSION with PRESSURE
       case sensor_t::PRESSION:     // when we send pressure we read button
       //TODO parse button
       break;
    }
  
    // if I sent the last sensor, reset current sensor to first one.
    if (++s > sensor_t::Last)
      s = sensor_t::First;

    process = true;
}
