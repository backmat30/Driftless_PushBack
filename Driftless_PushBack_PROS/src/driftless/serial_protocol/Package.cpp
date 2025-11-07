#include "driftless/serial_protocol/Package.hpp"

namespace driftless::serial_protocol {
void Package::addPacket(const Packet& packet) {
  m_packets.push_back(packet);
}

void Package::addPacket(const ESerialKey key, const std::vector<uint8_t>& value) {
  Packet packet{key, value};
  m_packets.push_back(packet);
}

void Package::clearPackets() {
  m_packets.clear();
}

std::vector<uint8_t> Package::getSerializedPackage() const {
  std::vector<uint8_t> output_vector;
  output_vector.push_back(0); // Placeholder for package size
  output_vector.push_back(static_cast<uint8_t>(m_packets.size()));
  for (const Packet& packet : m_packets) {
    output_vector.push_back(static_cast<uint8_t>(packet.key));
    output_vector.insert(output_vector.end(), packet.value.begin(), packet.value.end());
  }

  output_vector[0] = static_cast<uint8_t>(output_vector.size() + 2); // +2 for CRC bytes
  return output_vector;
}
}