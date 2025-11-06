#ifndef __PACKAGE_HPP__
#define __PACKAGE_HPP__

#include <vector>

#include "driftless/serial_protocol/Packet.hpp"

namespace driftless {
  namespace serial_protocol {
    class Package {
      private:
        std::vector<Packet> m_packets;
      
      public:

        void addPacket(const Packet& packet);

        void addPacket(const ESerialKey key, const std::string& value);

        void clearPackets();

        std::string getSerializedPackage() const;
    };
  }
}
#endif