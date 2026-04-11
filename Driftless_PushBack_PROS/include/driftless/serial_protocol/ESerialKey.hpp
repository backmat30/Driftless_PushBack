#ifndef __E_SERIAL_KEY_HPP__
#define __E_SERIAL_KEY_HPP__
#include <cstdint>
/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {
/// @brief Namespace for serial communication protocols
/// @author Matthew Backman
namespace serial_protocol {
/// @brief Enumeration representing valid keys for serial communication
/// @author Matthew Backman
enum class ESerialKey : int8_t {
  REQUEST_PACKET = 'R',
  CALIBRATE_ODOM = 'C',
  XPOS = 'X',
  YPOS = 'Y',
  THETA = 'H',
  ACK = 'A',
  ERROR = 'E',
  POSITION = 'P'
};
}  // namespace serial_protocol
}  // namespace driftless
#endif