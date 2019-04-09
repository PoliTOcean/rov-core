#include "SPIInterface.h"
#include "Arduino.h"


volatile SPIInterface *objPtr = NULL;

void SPIInterface::configure(){  
  if(objPtr!=NULL)
    return;
    
  cli();
  SPCR = 0b11000000;
  sei();

  flag_reset = true;

  objPtr = this;
}

void SPIInterface::sendBack(char value){
  
    rec = SPDR;        // load the recieved value
    
    if(flag_reset){
      // set the value that has to be sended
      sen = SPDR;
      // send back the value
      SPDR = sen;              
      flag_reset = false;
    }else{
      // set the value that has to be sended
      sen = 0x11;  
      // send back the value      
      SPDR = sen; 
      flag_reset = false;       
    }
}

void SPIInterface::setSen(char value){
  sen = value;
}

void SPIInterface::setRec(char value){
  rec = value;
}

// SPI interrupt routine
ISR (SPI_STC_vect){
  byte value = SPDR, newValue = 0x12;

  SPDR = newValue;
  //if(objPtr!=NULL)
    //objPtr->sendBack(0x11);
}
