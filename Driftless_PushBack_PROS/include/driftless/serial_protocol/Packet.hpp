#ifndef __PACKET_HPP__
#define __PACKET_HPP__

#include <vector>
#include "driftless/serial_protocol/ESerialKey.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for serial communication protocols
/// @author Matthew Backman
namespace serial_protocol {

/// @brief Represents a packet of data for serial communication
/// @author Matthew Backman
struct Packet {
  /// @brief Key for the packet, what the data represents
  ESerialKey key;
  /// @brief Value of the packet
  std::vector<uint8_t> value;

  bool operator==(const Packet& other) const {
    return key == other.key && value == other.value;
  }
};
}  // namespace serial_protocol
}  // namespace driftless
#endif