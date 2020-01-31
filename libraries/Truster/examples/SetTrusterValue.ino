#include "Arduino.h"
#include "Truster.h"

Truster truster;

void setup()
{
    Serial.begin(9600);

    cli();              //stop interrupts
    truster.attach(11); //attach motor
    sei();              //allow interrupts
}

void loop()
{
    if (Serial.available())
        truster.setValue(Serial.parseInt());
}
