/**
 * @author: pettinz
 */

#ifndef COMMANDS_H
#define COMMANDS_H

using namespace std;

namespace Politocean
{
namespace Constants
{
namespace Commands
{

namespace ATMega
{
namespace Axes
{
const short X_AXIS = 0;
const short Y_AXIS = 1;
const short Z_AXIS = 2;
const short RZ_AXIS = 3;
const short PITCH_AXIS = 4;
} // namespace Axes

namespace SPI
{
const unsigned char VDOWN_ON = 0x04;
const unsigned char VDOWN_OFF = 0x05;
const unsigned char VUP_ON = 0x06;
const unsigned char VUP_OFF = 0x07;
const unsigned char FAST = 0x0D;
const unsigned char SLOW = 0x0E;
const unsigned char MEDIUM = 0x0C;
const unsigned char START_AND_STOP = 0x12;
const unsigned char VUP_FAST_ON = 0x13;
const unsigned char VUP_FAST_OFF = 0x14;
const unsigned char PITCH_CONTROL = 0x15;

namespace Delims
{
const unsigned char AXES = 0xFF;
const unsigned char COMMAND = 0x00;
const unsigned char SENSORS = 0xFF;
} // namespace Delims
} // namespace SPI
} // namespace ATMega

} // namespace Commands
} // namespace Constants
} // namespace Politocean

#endif //COMMANDS_H
