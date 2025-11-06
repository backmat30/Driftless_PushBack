#include "driftless/serial_protocol/Package.hpp"

namespace driftless::serial_protocol {
void Package::addPacket(const Packet& packet) {
  m_packets.push_back(packet);
}

void Package::addPacket(const ESerialKey key, const std::string& value) {
  Packet packet{key, value};
  m_packets.push_back(packet);
}

void Package::clearPackets() {
  m_packets.clear();
}

std::string Package::getSerializedPackage() const {
  std::string output_string{};
  output_string += static_cast<char>(m_packets.size());
  for(Packet packet : m_packets) {
    output_string += static_cast<char>(packet.key);
    output_string += packet.value;
  }
  return output_string;
}
}