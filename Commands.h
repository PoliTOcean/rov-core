/**
 * Commands
 * PolitTOcean year 2019
 */
 
#ifndef POLITOCEAN_COMMANDS_H
#define POLITOCEAN_COMMANDS_H

namespace Politocean {
namespace Constants {
namespace Commands {
namespace ATMega
{
  namespace Axis
  {
      const short X_AXES    = 0;
      const short Y_AXES    = 1;
      const short RZ_AXES   = 2;
      const short UP_AXES   = 3;//the number define here need to be confirmed!!
      const short DOWN_AXES = 4;//the number define here need to be confirmed!!
  }

  namespace SPI
  {
      const unsigned char VDOWN_ON           = 0x04;
      const unsigned char VDOWN_OFF          = 0x05;
      const unsigned char VUP_ON             = 0x06;
      const unsigned char VUP_OFF            = 0x07;
      const unsigned char FAST               = 0x0D;
      const unsigned char SLOW               = 0x0E;
      const unsigned char MEDIUM             = 0x0C;
      const unsigned char START_AND_STOP     = 0x12;
      const unsigned char VUP_FAST_ON        = 0x13;
      const unsigned char VUP_FAST_OFF       = 0x14;

      namespace Delims
      {
          const unsigned char AXES         = 0xFF;
          const unsigned char COMMAND      = 0x00;
          const unsigned char SENSORS      = 0xFF;
      }
  }
}
}
}
}
#endif
