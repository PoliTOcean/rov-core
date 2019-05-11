/**
 * Commands
 * PolitTOcean year 2019
 */
 
#ifndef POLITOCEAN_COMMANDS_H
#define POLITOCEAN_COMMANDS_H
namespace Commands
{
  namespace Buttons
  {
    const int VDOWN             = 5;
    const int MOTORS            = 1;
    const int WRIST             = 7;
    const int RESET             = 9;
    const int VUP               = 14;
    const int MEDIUM_FAST       = 24;
    const int SLOW              = 25;
    const int AUTONOMOUS        = 66;
    const int START_AND_STOP    = 2;
  }
  
  namespace Axes
  {
    static const int X          = 0;
    static const int Y          = 1;
    static const int SHOULDER   = 2;
    static const int WRIST      = 4;
    static const int RZ         = 5;
  }
  
  namespace Actions
  {
    const unsigned char MOTORS_SWAP        = 0x01;
    const unsigned char MOTORS_ON          = 0x02;
    const unsigned char MOTORS_OFF         = 0x03;
    const unsigned char VDOWN_ON           = 0x04;
    const unsigned char VDOWN_OFF          = 0x05;
    const unsigned char VUP_ON             = 0x06;
    const unsigned char VUP_OFF            = 0x07;
    const unsigned char WRIST_SWAP         = 0x08;
    const unsigned char WRIST_ON           = 0x09;
    const unsigned char WRIST_OFF          = 0x0A;
    const unsigned char RESET              = 0x0B;
    const unsigned char MEDIUM             = 0x0C;
    const unsigned char FAST               = 0x0D;
    const unsigned char SLOW               = 0x0E;
    const unsigned char AUTONOMOUS_ON      = 0x10;
    const unsigned char AUTONOMOUS_OFF     = 0x11;
    const unsigned char START_AND_STOP     = 0x12;
    const unsigned char VUP_FAST_ON        = 0x13;
    const unsigned char VUP_FAST_OFF       = 0x14;
    
    const unsigned char NONE               = 0x00;
  }

  namespace Spi {
    const unsigned char AXES_DELIM         = 0xFF;
    const unsigned char COMMAND_DELIM      = 0x00;
    const unsigned char SENSORS_DELIM      = 0xFF;
  }

  
}
#endif
