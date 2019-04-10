#include "SPIInterface.h"
#include "Arduino.h"
#include <SPI.h>

void SPIInterface::configure(){  
  pinMode(MISO, OUTPUT);
  SPCR |= _BV(SPE);

  // Setup SPDR to send 0xFF
  SPDR = 0xFF;

  // Setup starting number to 0
  i = 0;
  process = false;

  SPI.attachInterrupt();
}

int SPIInterface::getI(){
  return i;
}

void SPIInterface::setI(int value){
  i = value;
}

byte SPIInterface::getR(){
  return r;
}

void SPIInterface::setR(byte value){
  r = value;
}

boolean SPIInterface::getProcess(){
  return process;
}

void SPIInterface::setProcess(boolean value){
  process = value;
}
