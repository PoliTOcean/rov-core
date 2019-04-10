/**
 * SPI interface 
 * Politocean 2019
 */

 #include "Arduino.h"

 #ifndef POLITOCEAN_SPIINTERFACE_H
 #define POLITOCEAN_SPIINTERFACE_H

 class SPIInterface{
  public:
    void configure();
    byte getR();
    void setR(byte value);
    boolean getProcess();
    void setProcess(boolean value);
    int getI();
    void setI(int value);
  private:
    volatile int i;
    volatile byte r;
    volatile boolean process;
 };

 #endif
