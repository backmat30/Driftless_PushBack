#include "driftless/serial_protocol/SerialProtocol.hpp"

namespace driftless::serial_protocol {
  template <typename T>
  std::string SerialProtocol::encode(const ESerialKey key, const T& value) {
    // converts to a string in the format "/<size><key><value>"
    std::string output{};
    output += START_DELIMITER;  // Start delimiter
    output += std::to_string(sizeof(T) + 1);  // Size of the value + the key
    output += static_cast<int8_t>(key);  // Key as a byte
    
    std::string value_str{};
    std::memcpy(value_str.data(), &value, sizeof(T));  // Copy the value bytes
    output += value_str;  // Append the value to the output string

    return output;
  }

  Packet SerialProtocol::decode(std::string& packet_info) {
    uint8_t size{};
    ESerialKey key{};
    std::string value{};

    if(hasValidPacket(packet_info)) {
      packet_info.erase(0, packet_info.find(START_DELIMITER));
      // Extract the size of the packet
      size = static_cast<uint8_t>(packet_info[1]);
      key = static_cast<ESerialKey>(packet_info[2]);
      value = packet_info.substr(3, size - 1);  // Extract the value
    }

    Packet packet{key, value};
    return packet;
  }

  bool SerialProtocol::hasValidPacket(const std::string& input_stream) const {
    bool has_valid_packet{true};
    if(input_stream.find(START_DELIMITER) == std::string::npos) {
      has_valid_packet = false;
    } else if (input_stream.size() < input_stream.find(START_DELIMITER) + 2) {
      // Check if the packet is too short to be valid
      has_valid_packet = false;
    } else {
      uint8_t size = static_cast<uint8_t>(input_stream[input_stream.find(START_DELIMITER) + 1]);
      if( input_stream.size() < input_stream.find(START_DELIMITER) + size + 2) {
        // Check if the packet is too short to contain the expected data
        has_valid_packet = false;
      }
    }
    return has_valid_packet;
  }
}