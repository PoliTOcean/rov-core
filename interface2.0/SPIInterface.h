/**
 * SPI interface 
 * Politocean 2019
 */

 #ifndef POLITOCEAN_SPIINTERFACE_H
 #define POLITOCEAN_SPIINTERFACE_H

 class SPIInterface{
  public:
    void configure();
    void sendBack(char value);
    void setSen(char value);
    void setRec(char value);
  private:
    volatile char rec;
    volatile char sen;
    volatile bool flag_reset;
 };

 #endif
