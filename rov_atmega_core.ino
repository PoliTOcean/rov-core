#include <SPI.h>
#include "IMU.h"
#include "Sensor.h"
#include "PressureSensor.h"
#include "Engine.h"
#include "Commands.h"
#include <RBD_Timer.h>
#include <DallasTemperature.h>

#define SENSORS_SIZE static_cast<int>(sensor_t::Last) + 1

#define dt 0.012 //12ms -> IMU needs to be calibrated with this dt

using namespace Politocean::Constants::Commands;

DallasTemperature seno(A1);

volatile byte sensors[SENSORS_SIZE];
volatile float currentPressure;
float temperature;

volatile Engine engine(dt); // engine manager

IMU imu(dt);     // imu sensor
MS5837 brSensor; // pressure sensor
RBD::Timer timer;

void setup()
{
    Serial.begin(9600); // initialize comunication via the serial port

    imu.configure(); // initialize IMU sensor

    delay(100);

    brSensor.setModel(MS5837::MS5837_02BA);
    brSensor.setFluidDensity(997); // kg/m^3 (freshwater, 1029 for seawater)
    brSensor.init();               // initialize pressure sensor

    delay(100);

    /** engine INIT **/
    engine.configure(); // initialize engine

    delay(1000); // delay of 1.5 seconds to make actions complete

    /** SPI SETUP **/
    cli();
    pinMode(MISO, OUTPUT); // SPI setup
    SPCR |= _BV(SPE);
    SPDR = 0xFF;           // set the SPI data register to 0xFF before sending sensors data
    SPI.attachInterrupt(); // enable SPI
    sei();

    timer.setTimeout(dt * 1000);
    timer.restart();

    seno.begin(); //porc
    seno.setWaitForConversion(false);
    seno.requestTemperaturesByIndex(0);
}

void sensorsRead()
{
    if (seno.isConversionComplete())
    {
        temperature = seno.getTempCByIndex(0);
        seno.requestTemperaturesByIndex(0);
    }
    brSensor.read();
    currentPressure = brSensor.pressure();
    imu.imuRead();
    imu.complementaryFilter();
}

void sensorsPrepare()
{
    sensors[static_cast<int>(sensor_t::TEMPERATURE_PWR)] = static_cast<byte>(temperature);
    sensors[static_cast<int>(sensor_t::PRESSURE)] = static_cast<byte>(currentPressure - 980);
    sensors[static_cast<int>(sensor_t::PITCH)] = static_cast<byte>((imu.roll + 3.15) * 10);
    sensors[static_cast<int>(sensor_t::ROLL)] = static_cast<byte>((imu.pitch + 3.15) * 10);
    sensors[static_cast<int>(sensor_t::TEMPERATURE_INT)] = static_cast<byte>(imu.temperature);
}

void loop()
{
    if (timer.onRestart())
    {
        //long now = micros();
        sensorsRead();

        sensorsPrepare();

        engine.evaluateHorizontal();

        //IMU's pitch is ROV's roll and viceversa
        engine.evaluateVertical(currentPressure, imu.pitch, imu.roll);
        // imu.printValues();
        // Serial.println(micros()-now);
    }
}

ISR(SPI_STC_vect)
{
    static byte c;
    static bool nextIsCommand = false, nextIsAxes = false, sensorsTerminator = false;
    static int axis = 0;
    static sensor_t s = sensor_t::First; // sensor counter

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
        SPDR = sensors[static_cast<int>(s)];

        // if I sent the last sensor, reset current sensor to first one.
        if (s++ == sensor_t::Last)
            sensorsTerminator = true;
        else
            ++s;
    }

    // check received data
    if (c == ATMega::SPI::Delims::COMMAND)
    {
        //the next incoming data is a button
        nextIsCommand = true;
        return;
    }
    else if (c == ATMega::SPI::Delims::AXES)
    {
        nextIsAxes = true;
        return;
    }

    if (nextIsCommand)
    {
        // process the nextIsCommand
        switch (c)
        {
        case ATMega::SPI::START_AND_STOP:
            if (engine.isStarted())
                engine.stop();
            else
                engine.start();
            break;
        case ATMega::SPI::VDOWN_ON:
            engine.goDown();
            break;
        case ATMega::SPI::VDOWN_OFF:
            engine.stopDown();
            break;
        case ATMega::SPI::VUP_ON:
            engine.goUp();
            break;
        case ATMega::SPI::VUP_OFF:
            engine.stopUp();
            break;
        case ATMega::SPI::VUP_FAST_ON:
            engine.goUpFast();
            break;
        case ATMega::SPI::VUP_FAST_OFF:
            engine.stopUpFast();
            break;
        case ATMega::SPI::FAST:
            engine.setPower(Engine::PowerMode::FAST);
            break;
        case ATMega::SPI::MEDIUM:
            engine.setPower(Engine::PowerMode::MEDIUM);
            break;
        case ATMega::SPI::SLOW:
            engine.setPower(Engine::PowerMode::SLOW);
            break;
        case ATMega::SPI::PITCH_CONTROL:
            if (engine.isPitchControlEnabled())
                engine.pitchControlOff();
            else
                engine.pitchControlOn();
            break;
        }
        nextIsCommand = false; // last command
        axis = 0;              // restart from x
    }
    else if (nextIsAxes)
    {
        switch (axis)
        {
        case ATMega::Axes::X_AXIS: //  read x
            engine.setX(c - 127);
            break;

        case ATMega::Axes::Y_AXIS: // read y
            engine.setY(c - 127);
            break;

        case ATMega::Axes::RZ_AXIS: //  read rz
            engine.setYaw(c - 127);
            break;

        case ATMega::Axes::PITCH_AXIS:
            engine.setPitchControlPower(c - 127);
            break;
        }

        if (++axis > 3)
        {
            nextIsAxes = false;
            axis = 0;
        }
    }
}
