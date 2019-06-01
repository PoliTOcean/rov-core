#include <SPI.h>
#include "IMU.h"
#include "Sensor.h"
#include "PressureSensor.h"
#include "Motors.h"
#include "Commands.h"
#include "RBD_Timer.h"

#define SENSORS_SIZE static_cast<int>(sensor_t::Last)+1

#define dt 0.012    //12ms -> IMU needs to be calibrated with this dt

volatile byte sensors[SENSORS_SIZE];
volatile float currentPressure;
float temperature;

IMU imu(dt);  // imu sensor
MS5837 brSensor;  // pressure sensor
Motors motors(dt);  // motors manager
RBD::Timer timer;

using namespace Politocean::Constants::Commands;

void setup() {    
    Serial.begin(9600);                   // initialize comunication via the serial port

    imu.configure();                      // initialize IMU sensor

    delay(100);
    
    brSensor.setModel(MS5837::MS5837_02BA);
    brSensor.setFluidDensity(997);        // kg/m^3 (freshwater, 1029 for seawater)
    brSensor.init();                      // initialize pressure sensor

    delay(100);

    /** MOTORS INIT **/
    motors.configure();                   // initialize motors
    
    delay(1000);                          // delay of 1.5 seconds to make actions complete

    /** SPI SETUP **/
    cli();
    pinMode(MISO, OUTPUT);                // SPI setup
    SPCR |= _BV(SPE);
    SPDR = 0xFF;                          // set the SPI data register to 0xFF before sending sensors data
    SPI.attachInterrupt();                // enable SPI
    sei();

    timer.setTimeout(dt*1000);
    timer.restart();
}

void sensorsRead(){
  temperature = analogRead(A1)/2.046;
  brSensor.read();
  currentPressure = brSensor.pressure();
  imu.imuRead();
  imu.complementaryFilter();
}

void sensorsPrepare(){
  sensors[static_cast<int>(sensor_t::TEMPERATURE_PWR)]  = static_cast<byte>( temperature );
  sensors[static_cast<int>(sensor_t::PRESSURE)]         = static_cast<byte>( currentPressure - 980 );
  sensors[static_cast<int>(sensor_t::PITCH)]            = static_cast<byte>( ( imu.pitch + 3.15 )*10 );
  sensors[static_cast<int>(sensor_t::ROLL)]             = static_cast<byte>( ( imu.roll + 3.15 )*10 );
  sensors[static_cast<int>(sensor_t::TEMPERATURE_INT)]  = static_cast<byte>( imu.temperature );
}

void loop() { 
  if( timer.onRestart() ){
   //long now = micros();
    sensorsRead();
  
    sensorsPrepare();
    
    motors.evaluateHorizontal();

    //IMU's pitch is ROV's roll and viceversa
    motors.evaluateVertical(currentPressure, imu.pitch, imu.roll);
   // imu.printValues();
   // Serial.println(micros()-now);
  }
  
}

ISR (SPI_STC_vect)
{
    static Motors* motors_ = &motors;
    static byte c;
    static bool nextIsCommand = false, nextIsAxes = false, sensorsTerminator = false;
    static int axis = 0;
    static sensor_t s = sensor_t::First;  // sensor counter
    
    c = SPDR;

    // check data to send
    if (sensorsTerminator)
    {
      SPDR = ATMega::SPI::Delims::SENSORS;
      sensorsTerminator = false;
      s = sensor_t::First;
    }
    else
    {
      // Prepare the next sensor's value to send through SPI
      SPDR = sensors[ static_cast<int>( s ) ];
      
      // if I sent the last sensor, reset current sensor to first one.
      if (s == sensor_t::Last)
      {
        sensorsTerminator = true;
      }
      else
      {
        ++s;
      }
    }

    // check received data
    if (c == ATMega::SPI::Delims::COMMAND)
    {
      //the next incoming data is a button
      nextIsCommand=true;
      return;
    }
    else if (c == ATMega::SPI::Delims::AXES)
    {
      nextIsAxes = true;
      return;
    }
    

    if(nextIsCommand){      
      // process the nextIsCommand
      switch(c){
        case ATMega::SPI::START_AND_STOP:
          if (motors_->started)
            motors_->stop();
          else
            motors_->start();
        break;
        case ATMega::SPI::VDOWN_ON:
          motors_->goDown();
        break;
        case ATMega::SPI::VDOWN_OFF:
          motors_->stopDown();
        break;
        case ATMega::SPI::VUP_ON:
          motors_->goUp();
        break;
        case ATMega::SPI::VUP_OFF:
          motors_->stopUp();
        break;
        case ATMega::SPI::VUP_FAST_ON:
          motors_->goUpFast();
        break;
        case ATMega::SPI::VUP_FAST_OFF:
          motors_->stopUpFast();
        break;
        case ATMega::SPI::FAST:
          motors_->setPower(Motors::FAST);
        break;
        case ATMega::SPI::MEDIUM:
          motors_->setPower(Motors::MEDIUM);
        break;
        case ATMega::SPI::SLOW:
          motors_->setPower(Motors::SLOW);
        break;
       }
      nextIsCommand = false; // last command
      axis = 0; // restart from x
    }
    else if (nextIsAxes)
    {
      switch(axis){
       case ATMega::Axis::X_AXES:         //  read x
        motors_->setX(c-127);
       break;
      
       case ATMega::Axis::Y_AXES:        // read y
       motors_->setY(c-127);
       break;
      
       case ATMega::Axis::RZ_AXES:       //  read rz
       motors_->setRz(c-127);
       break;
      }
      
      if (++axis > 2){
        nextIsAxes = false;
        axis = 0;
      }
    }
}
