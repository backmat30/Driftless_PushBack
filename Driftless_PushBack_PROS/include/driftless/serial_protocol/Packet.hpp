#ifndef __PACKET_HPP__
#define __PACKET_HPP__

#include "driftless/serial_protocol/ESerialKey.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for serial communication protocols
/// @author Matthew Backman
namespace serial_protocol {

/// @brief Represents a packet of data for serial communication
/// @author Matthew Backman
class Packet {
  /// @brief Key for the packet, what the data represents
  ESerialKey key;
  /// @brief Value of the packet
  int8_t* value;
};
}  // namespace serial_protocol
}  // namespace driftless
#endif