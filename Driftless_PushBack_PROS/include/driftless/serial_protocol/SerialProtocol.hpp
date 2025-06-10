#ifndef __I_SERIAL_PROTOCOL_HPP__
#define __I_SERIAL_PROTOCOL_HPP__

#include <string>
#include <utility>

#include "driftless/serial_protocol/ESerialKey.hpp"
#include "driftless/serial_protocol/Packet.hpp"

namespace driftless {
namespace serial_protocol {
class SerialProtocol {
  private:
  /// @brief Start delimiter for the packet protocol
  static constexpr char START_DELIMITER{'/'};
 public:
  /// @brief Destroys the Serial Protocol object
  virtual ~SerialProtocol() = default;

  /// @brief Encodes a key-value pair into a packet string for serial
  /// communication
  /// @tparam T The type of the value
  /// @param key __std::string&__ The key
  /// @param value __T__ The value
  /// @return __std::string__ The key-value pair formatted to the specified
  /// packet protocol
  template <typename T>
  std::string encode(const ESerialKey key, const T value);

  /// @brief Decodes a packet from a serial transmission into a key-value pair
  /// @param packet_info __std::string&__ The packet to decode
  /// @return __Packet__ The key-value pair
  Packet decode(std::string& packet_info);

  /// @brief Determines if an input stream contains a valid packet
  /// @param input_stream __std::string&__ The input stream to check
  /// @return __bool__ True if the input stream contains a valid packet,
  /// false otherwise
  bool hasValidPacket(const std::string& input_stream) const;
};
}  // namespace serial_protocol
}  // namespace driftless
#endif