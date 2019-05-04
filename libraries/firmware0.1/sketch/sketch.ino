
#include <Sensor.h>
#include <Array.h>
#include <SPI.h>
#include "IMU.h"
#include "PressureSensor.h"
#include "Motors.h"
#include "Commands.h"

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
  sensors[static_cast<int>(sensor_t::PITCH)].setValue(imu.getPitch());
  sensors[static_cast<int>(sensor_t::ROLL)].setValue(imu.getRoll());
}

ISR (SPI_STC_vect)
{
    c = SPDR;
    
    // Prepare the next sensor's value to send through SPI
    SPDR = sensors[static_cast<int>(s)].getValue();

    switch(s){
       case sensor_t::ROLL:         // when we send roll we read x
       motors.x = c-127;
       break;
      
       case sensor_t::PITCH:        // when we send pitch we read y
       motors.y = c-127;
       break;
      
       case sensor_t::TEMPERATURE:  // when we send temperature we read rz
       motors.rz = c-127;
       break;

       // TODO change PRESSION with PRESSURE
       case sensor_t::PRESSION:     // when we send pressure we read button
       byte selector = c;
       switch(selector){
        case MOTORS_ON:
          motors.start();
        break;
        case MOTORS_OFF:
          motors.stop();
        break;
        case VDOWN:
          motors.goDown();
        break;
        case VDOWN_STOP:
          motors.stopVertical();
        break;
        case VUP:
          motors.goUp();
        break;
        case VUP_STOP:
          motors.stopVertical();
        break;
        case FAST:
          motors.velocity = 3;
        break;
        case NORMAL:
          motors.velocity = 2;
        break;
        case SLOW:
          motors.velocity = 1;
        break;
       }
       
       break;
    }
  
    // if I sent the last sensor, reset current sensor to first one.
    if (++s > sensor_t::Last)
      s = sensor_t::First;

    process = true;
}
