
#include "SPIInterface.h"
//#include <SPI.h>

volatile int i;
volatile byte r;
volatile boolean process;

volatile SPIInterface interface;

void setup() {

  Serial.begin(9600);
  Serial.println("Reboot...");
  
  Serial.println("Configure spi library...");
  interface.configure();
}

void loop() {
  // put your main code here, to run repeatedly:
  if (interface.getProcess()) {
    Serial.println(interface.getR());
    interface.setProcess(false);
  }
}

ISR (SPI_STC_vect)
{
  interface.setR(SPDR);
  interface.setProcess(true);
  SPDR = interface.getI()+1; 
  interface.setI(interface.getI()+1);
}
