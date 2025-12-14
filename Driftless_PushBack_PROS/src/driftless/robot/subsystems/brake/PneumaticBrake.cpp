#include "driftless/robot/subsystems/brake/PneumaticBrake.hpp"

namespace driftless::robot::subsystems::brake {
  void PneumaticBrake::init() {}

  void PneumaticBrake::run() {}

  void PneumaticBrake::deploy() {
    m_brake_piston->extend();
    m_is_deployed = true;
  }

  void PneumaticBrake::retract() {
    m_brake_piston->retract();
    m_is_deployed = false;
  }

  bool PneumaticBrake::isDeployed() {
    return m_is_deployed;
  }

  void PneumaticBrake::setBrakePiston(std::unique_ptr<io::IPiston>& brake_piston) {
    m_brake_piston = std::move(brake_piston);
  }
}