#ifndef __PACKAGE_HPP__
#define __PACKAGE_HPP__

#include <cstring>
#include <vector>

#include "driftless/serial_protocol/Packet.hpp"

namespace driftless {
namespace serial_protocol {
class Package {
 private:
  std::vector<Packet> m_packets;

 public:
  void addPacket(const Packet& packet);

  void addPacket(const ESerialKey key, const std::vector<uint8_t>& value);

  void clearPackets();

  std::vector<uint8_t> getSerializedPackage() const;
};
}  // namespace serial_protocol
}  // namespace driftless
#endif