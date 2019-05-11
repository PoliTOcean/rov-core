#include <Array.h>
#include <SPI.h>
#include "IMU.h"
#include "Sensor.h"
#include "PressureSensor.h"
#include "Motors.h"
#include "Commands.h"
#include "RBD_Timer.h"

#define SENSORS_SIZE static_cast<int>(sensor_t::Last)+1

volatile Array<Sensor<byte>, SENSORS_SIZE> sensors; // array of sensors

volatile float currentPressure;
volatile bool updatedAxis = false;

float temperature;

IMU imu;  // imu sensor

MS5837 brSensor;  // pressure sensor
Motors motors;  // motors manager

RBD::Timer timer;

using namespace Commands;

long now;

void setup() {
   // analogReference(INTERNAL);
    
    Serial.begin(9600);                   // initialize comunication via the serial port

    /** SENSORS CONFIGURATION **/
    for (auto sensor_type : sensor_t()) // create sensors array
        sensors.push_back(Sensor<byte>(sensor_type, 0));

    imu.configure();                      // initialize IMU sensor

    delay(1000);
    
    brSensor.setModel(MS5837::MS5837_02BA);
    brSensor.setFluidDensity(997);        // kg/m^3 (freshwater, 1029 for seawater)
    brSensor.init();                      // initialize pressure sensor

    delay(1000);

    /** MOTORS INIT **/
    motors.configure(imu);                // initialize motors
    
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

    now = micros();
}

void sensorsRead(){
  temperature = analogRead(A1)/2.046;
  brSensor.read();
  currentPressure = brSensor.pressure();
  imu.imuRead();
  imu.complementaryFilter();
}

void sensorsPrepare(){

/*
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

  sensors[static_cast<int>(sensor_t::TEMPERATURE)].setValue( static_cast<byte>( temperature ) );
  sensors[static_cast<int>(sensor_t::PRESSURE)].setValue( static_cast<byte>( currentPressure - 980 ) );
  sensors[static_cast<int>(sensor_t::PITCH)].setValue( static_cast<byte>(  ( imu.pitch + 3.15 )*10 ) );
  sensors[static_cast<int>(sensor_t::ROLL)].setValue( static_cast<byte>(   ( imu.roll + 3.15 )*10 ) );
}

void loop() {
  // prepare data to send back via spi
 // unsigned long now = micros();

  if( timer.onRestart() ){    
    sensorsRead();
  
    sensorsPrepare();
    
    if(updatedAxis){
      motors.evaluateHorizontal();
      updatedAxis=false;
    }
    motors.evaluateVertical(currentPressure);
  }
  //Serial.println((float)analogRead(A0) / (float)2.046);
 // now = micros()-now;
 // Serial.println((float)now/1000);
}

ISR (SPI_STC_vect)
{
    static Motors* motors_ = &motors;
    static byte c;
    static bool nextIsButton = false, nextIsAxes = false, sensorsTerminator = false;
    static int axis = 0;
    static sensor_t s = sensor_t::First;  // sensor counter
    
    c = SPDR;

    // check data to send
    if(sensorsTerminator){
      SPDR = 0xFF;
      sensorsTerminator = false;
      s = sensor_t::First;
    }else{
      // Prepare the next sensor's value to send through SPI
      SPDR = sensors[static_cast<int>(s)].getValue();
      // if I sent the last sensor, reset current sensor to first one.
      if (++s > sensor_t::Last)
        sensorsTerminator = true;
    }

    // check received data
    if(c == 0x00){
      //the next incoming data is a button
      nextIsButton=true;
      return;
    }
    else if (c == 0xFF)
    {
      nextIsAxes = true;
      s = sensor_t::First;
      return;
    }
    

    if(nextIsButton){      
      // process the nextIsButton
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
      nextIsButton = false; // last command
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
