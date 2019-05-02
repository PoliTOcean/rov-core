
#include <Sensor.h>
#include <Array.h>
#include <SPI.h>
#include "IMU.h"
#include "PressureSensor.h"
#include "Motors.h"

#define SENSORS_SIZE static_cast<int>(sensor_t::Last)+1

/**
 * @s: current sensor to be sent via SPI.
 * @sensors: array of Sensor objects with byte values.
 */
volatile sensor_t s;
volatile Array<Sensor<byte>, SENSORS_SIZE> sensors;

/**
 * @process     : it is set to `true` when by the SPI ISR when it ends
 * @c           : it stores SPI Data Register (SPDR) value, data received via SPI
 * @lastButton  : it stores the last button data as follows:
 *                  (*) index for the last button identifier
 *                  (*) value for the last button value
 */
volatile bool process;
volatile byte c;
volatile int lastButton[] = { -1, -1 };

IMU imu;
PressureSensor pressure;
Motors motors;


void setup()
{
    // Initialize comunication system
    Serial.begin(9600);

    // Initialize IMU sensor
    imu.configure();

    // Initialize pressure sensor
    pressure.configure();
    
    // Initialize motors
    motors.configure(pressure,imu);
    
    // Delay of 1 second to make actions complete
    delay(1000);

    // SPI setup
    pinMode(MISO, OUTPUT);
    SPCR |= _BV(SPE);
    SPDR = 0xFF;
    SPI.attachInterrupt();
    
    // Create sensors array
    for (auto sensor_type : sensor_t())
        sensors.push_back(Sensor<byte>(sensor_type, 0));

    s = sensor_t::First;
}

void loop()
{
  //TODO: Prepare data to send via spi. Setup all the sensors.

  sensors[static_cast<int>(sensor_t::PITCH)].setValue(imu.pitch);
  sensors[static_cast<int>(sensor_t::ROLL)].setValue(imu.roll);
}

ISR (SPI_STC_vect)
{
    c = SPDR;
    
    // Prepare the next sensor's value to send through SPI
    SPDR = sensors[static_cast<int>(s)].getValue();

    switch(s)
    {
      case sensor_t::ROLL:
        // If we send roll value, we read x value
        motors.x = float(c-127)/127;
      break;
      
      case sensor_t::PITCH:
        // If we send pitch value, we read y value
        motors.y = float(c-127)/127;
      break;
      
      case sensor_t::TEMPERATURE:
        // If we send temperature value, we read rz value
        motors.rz = float(c-127)/127;
      break;

      // TODO change PRESSION with PRESSURE
      case sensor_t::PRESSION:
        // If we send pressure value, we read button value
        bool value = (c >> 7) & 0x01;
        unsigned short int id = c & 0x7F;

        // Check if button has not changed
        if (id == lastButton[0] && value == lastButton[1]) break;
      
        // Update lastButton with current button
        lastButton[0] = id; lastButton[1] = value;
        switch(id)
        {
          // Update value for button with id as identifier
        }

      break;
    }
  
    // If I sent the last sensor, reset current sensor to the first one.
    if (++s > sensor_t::Last)
      s = sensor_t::First;

    process = true;
}
