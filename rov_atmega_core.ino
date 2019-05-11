#include <SPI.h>
#include "IMU.h"
#include "Sensor.h"
#include "PressureSensor.h"
#include "Motors.h"
#include "Commands.h"
#include "RBD_Timer.h"

#define SENSORS_SIZE static_cast<int>(sensor_t::Last)+1

volatile byte sensors[SENSORS_SIZE];
volatile float currentPressure;
volatile bool updatedAxis = false;

float temperature;

IMU imu;  // imu sensor

MS5837 brSensor;  // pressure sensor
Motors motors;  // motors manager

RBD::Timer timer;

using namespace Commands;

void setup() {
   // analogReference(INTERNAL);
    
    Serial.begin(9600);                   // initialize comunication via the serial port

    imu.configure();                      // initialize IMU sensor

    delay(1000);
    
    brSensor.setModel(MS5837::MS5837_02BA);
    brSensor.setFluidDensity(997);        // kg/m^3 (freshwater, 1029 for seawater)
    brSensor.init();                      // initialize pressure sensor

    delay(1000);

    /** MOTORS INIT **/
    motors.configure();                // initialize motors
    
    delay(3000);                          // delay of 1 second to make actions complete

    /** SPI SETUP **/
    cli();
    pinMode(MISO, OUTPUT);                // SPI setup
    SPCR |= _BV(SPE);
    SPDR = 0xFF;                          // set the SPI data register to 0xFF before sending sensors data
    SPI.attachInterrupt();                // enable SPI
    sei();

    timer.setTimeout(IMU_dT*1000);
    timer.restart();
}

void sensorsRead(){
  static int readingCounters = 0;
  if(readingCounters > 30){
    temperature = analogRead(A1)/2.046;
    readingCounters = 0;
  }
  brSensor.read();
  currentPressure = brSensor.pressure();
  imu.imuRead();
  imu.complementaryFilter();
  readingCounters++;
}

void sensorsPrepare(){

/* DEBUG
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.print(" °C (");
  Serial.print((int)static_cast<byte>((int)temperature));
  Serial.print(")\tPressure: ");
  Serial.print(brSensor.pressure());
  Serial.print(" mBar (");
  Serial.print((int)static_cast<byte>((int)brSensor.pressure()/10));
  Serial.print(")  ");
  Serial.print(brSensor.depth());
  Serial.print(" m\tPitch: ");
  Serial.print(imu.pitch);
  Serial.print(" ° (");
  Serial.print((int)static_cast<byte>((int)imu.pitch));
  Serial.print(")\tRoll: ");
  Serial.print(imu.roll);
  Serial.print(" ° (");
  Serial.print((int)static_cast<byte>((int)imu.roll));
  Serial.println(")");*/

  sensors[static_cast<int>(sensor_t::TEMPERATURE)]  = static_cast<byte>( temperature );
  sensors[static_cast<int>(sensor_t::PRESSURE)]     = static_cast<byte>( currentPressure - 980 );
  sensors[static_cast<int>(sensor_t::PITCH)]        = static_cast<byte>( ( imu.pitch + 3.15 )*10 );
  sensors[static_cast<int>(sensor_t::ROLL)]         = static_cast<byte>( ( imu.roll + 3.15 )*10 );
}

void loop() {

  if( timer.onRestart() ){    
    sensorsRead();
  
    sensorsPrepare();
    
    if(updatedAxis){
      motors.evaluateHorizontal();
      updatedAxis=false;
    }
    motors.evaluateVertical(currentPressure, imu.roll, imu.pitch);
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
      SPDR = Spi::SENSORS_DELIM;
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
    if (c == Spi::COMMAND_DELIM)
    {
      //the next incoming data is a button
      nextIsCommand=true;
      return;
    }
    else if (c == Spi::AXES_DELIM)
    {
      nextIsAxes = true;
      return;
    }
    

    if(nextIsCommand){      
      // process the nextIsCommand
      switch(c){
        case Actions::START_AND_STOP:
          if (motors_->started)
            motors_->stop();
          else
            motors_->start(currentPressure);
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
          motors_->setPower(Motors::FAST);
        break;
        case Actions::MEDIUM:
          motors_->setPower(Motors::MEDIUM);
        break;
        case Actions::SLOW:
          motors_->setPower(Motors::SLOW);
        break;
       }
      nextIsCommand = false; // last command
      axis = 0; // restart from x
    }
    else if (nextIsAxes)
    {
      switch(axis){
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
      
      if (++axis > 2){
        nextIsAxes = false;
        axis = 0;
      }

      updatedAxis = true;
    }
}
