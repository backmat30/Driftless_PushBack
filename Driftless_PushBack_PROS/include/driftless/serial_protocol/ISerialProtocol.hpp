#ifndef __I_SERIAL_PROTOCOL_HPP__
#define __I_SERIAL_PROTOCOL_HPP__

#include <string>
#include <utility>

namespace driftless {
namespace serial_protocol {
class ISerialProtocol {
 public:
  /// @brief Destroys the Serial Protocol object
  virtual ~ISerialProtocol() = default;

  /// @brief Encodes a key-value pair into a packet string for serial
  /// communication
  /// @param key __std::string&__ The key
  /// @param value __std::string&__ The value
  /// @return __std::string__ The key=value pair formatted to the specified
  /// packet protocol
  virtual std::string encode(const std::string& key,
                             const std::string& value) = 0;

  /// @brief Decodes a packet from a serial transmission into a key-value pair
  /// @param packet __std::sttring&__ The packet to decode
  /// @return __std::pair<std::string, std::string>__ The key-value pair
  /// extracted from the packet
  virtual std::pair<std::string, std::string> decode(
      const std::string& packet) = 0;

  /// @brief Gets the character used to start a packet
  /// @return __char__ The start delimiter character
  virtual char getStartDelimiter() const = 0;

  /// @brief Gets the character used to end a packet
  /// @return __char__ The end delimiter character
  virtual char getEndDelimiter() const = 0;
};
}
}
#endif