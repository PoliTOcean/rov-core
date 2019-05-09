
#include <Array.h>
#include <SPI.h>
#include "IMU.h"
#include "Sensor.h"
#include "PressureSensor.h"
#include "Motors.h"
#include "Commands.h"

#define SENSORS_SIZE static_cast<int>(sensor_t::Last)+1

volatile sensor_t s;  // sensor counter
volatile Array<Sensor<byte>, SENSORS_SIZE> sensors; // array of sensors

volatile bool updatedAxis = false;
volatile byte c;
volatile bool nextIsButton = false;
volatile int receivedDataSelector = 0;

IMU imu;  // imu sensor

MS5837 brSensor;  // pressure sensor
Motors motors;  // motors manager

using namespace Commands;

void setup() {
   // analogReference(INTERNAL);
    
    Serial.begin(9600);                   // initialize comunication via the serial port

    /** SENSORS CONFIGURATION **/
    for (auto sensor_type : sensor_t()) // create sensors array
        sensors.push_back(Sensor<byte>(sensor_type, 0));
    s = sensor_t::First;              // set the sensor counter

    imu.configure();                      // initialize IMU sensor

    delay(1000);
    
    brSensor.setModel(MS5837::MS5837_30BA);
    brSensor.setFluidDensity(997);        // kg/m^3 (freshwater, 1029 for seawater)
    brSensor.init();                      // initialize pressure sensor

    delay(1000);

    /** MOTORS INIT **/
    motors.configure(brSensor, imu);       // initialize motors
    
    delay(3000);                          // delay of 1 second to make actions complete

    /** SPI SETUP **/
    cli();
    pinMode(MISO, OUTPUT);                // SPI setup
    SPCR |= _BV(SPE);
    SPDR = 0xFF;                          // set the SPI data register to 0xFF before sending sensors data
    SPI.attachInterrupt();                // enable SPI
    sei();
}

void sensorsRead(){
  brSensor.read();
  imu.imuRead();
  imu.complementaryFilter();
}

void sensorsPrepare(){
   float temperature = analogRead(A0)/2.046;
  sensors[static_cast<int>(sensor_t::TEMPERATURE)].setValue(static_cast<byte>(temperature));
  sensors[static_cast<int>(sensor_t::PRESSURE)].setValue(brSensor.pressure());
  sensors[static_cast<int>(sensor_t::PITCH)].setValue(imu.pitch);
  sensors[static_cast<int>(sensor_t::ROLL)].setValue(imu.roll);
}

void loop() {
  // prepare data to send back via spi
 // unsigned long now = micros();

  sensorsRead();

  sensorsPrepare();
  
  if(updatedAxis){
    motors.evaluateHorizontal();
    updatedAxis=false;
  }
  motors.evaluateVertical();

  //Serial.println((float)analogRead(A0) / (float)2.046);
 // now = micros()-now;
 // Serial.println((float)now/1000);
}

ISR (SPI_STC_vect)
{
    static Motors* motors_ = &motors;
    
    c = SPDR;
    
    // Prepare the next sensor's value to send through SPI
    SPDR = sensors[static_cast<int>(s)].getValue();

    // if I sent the last sensor, reset current sensor to first one.
    if (++s > sensor_t::Last)
      s = sensor_t::First;
      
    if(c == 0x00){
      //the next incoming data is a button
      nextIsButton=true;
      return;
    }
    

    if(nextIsButton){      
      // process the nextIsButton
      switch(c){
        case Actions::START_AND_STOP:
          if (motors_->started)
            motors_->stop();
          else
            motors_->start();
        break;
        case Actions::VDOWN_ON:
          motors_->goDown();
        break;
        case Actions::VDOWN_OFF:
          motors_->stopDown();
        break;
        case Actions::VUP_ON:
          motors_->goUp();
        break;
        case Actions::VUP_OFF:
          motors_->stopUp();
        break;
        case Actions::VUP_FAST_ON:
          motors_->goUpFast();
        break;
        case Actions::VUP_FAST_OFF:
          motors_->stopUpFast();
        break;
        case Actions::FAST:
          motors_->setPower(3);
        break;
        case Actions::MEDIUM:
          motors_->setPower(2);
        break;
        case Actions::SLOW:
          motors_->setPower(1);
        break;
       }
      nextIsButton = false; // last command
      receivedDataSelector = 0; // restart from x
    }else{
      switch(receivedDataSelector){
       case 0:         //  read x
        motors_->setX(c);
       break;
      
       case 1:        // read y
       motors_->setY(c);
       break;
      
       case 2:        //  read rz
       motors_->setRz(c);
       break;
      }
      
      if (++receivedDataSelector >= 3)
        receivedDataSelector = 0;

      updatedAxis = true;
    }
}
