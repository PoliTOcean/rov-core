/*
 * Created by pettinz.
 */

#include <Sensor.h>
#include <Array.h>
#include <SPI.h>

#define SENSORS_SIZE static_cast<int>(sensor_t::Last)+1

volatile sensor_t s; // current sensor
volatile Array<Sensor<byte>, SENSORS_SIZE> sensors;

volatile bool process, reset_;
volatile byte c;

byte current;

void SPISetup()
{
    reset = true;
    SPDR = 0xFF;
    process = false;
    s = sensor_t::First;
}

void setup() {
    Serial.begin(9600);
    
    pinMode(MISO, OUTPUT);
    SPCR |= _BV(SPE);
    
    // Reset SPI
    SPISetup();
    SPI.attachInterrupt();
    
    // create sensors array
    for (auto sensor_type : sensor_t())
        sensors.push_back(Sensor<byte>(sensor_type, 0));
}

void loop() {
    while (true)
        if (process) {
            Serial.print("Raspi sent me "), Serial.println(raspi);
            process = false;
        }
}

ISR (SPI_STC_vect)
{
    c = SPDR;
    
    // Prepare the next sensor's value to send through SPI
    SPDR = sensors[static_cast<int>(s)].getValue(); 
  
    sensors[static_cast<int>(i)].setValue(sensors[static_cast<int>(s)].getValue()+1);
  
    // if I sent the last sensor, reset current sensor to first one.
    if (++s > sensor_t::Last)
      s = sensor_t::First;

    process = true;
}
