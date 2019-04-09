#include "SPIInterface.h"
#include "FakeSensor.h"

SPIInterface interface;
FakeSensor sensor;

void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  Serial.println("Restart ATMega...");

  Serial.println("Configure SPI...");
  interface.configure();

  Serial.println("Configure FakeSensor...");
  sensor.configure();

}

void loop() {
  // put your main code here, to run repeatedly:-
}
/*
// SPI interrupt routine
ISR (SPI_STC_vect)
{
  if(objPtr!=NULL && sensorPtr!=NULL)
    objPtr->sendBack(sensorPtr->getValue());
}*/
