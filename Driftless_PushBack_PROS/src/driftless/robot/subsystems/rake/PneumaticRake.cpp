#include "driftless/robot/subsystems/rake/PneumaticRake.hpp"

namespace driftless::robot::subsystems::rake {
  void PneumaticRake::init() {}

  void PneumaticRake::run() {}

  void PneumaticRake::deploy() {
    m_rake_piston->extend();
    m_is_deployed = true;
  }

  void PneumaticRake::retract() {
    m_rake_piston->retract();
    m_is_deployed = false;
  }

  bool PneumaticRake::isDeployed() {
    return m_is_deployed;
  }

  void PneumaticRake::setRakePiston(std::unique_ptr<io::IPiston>& rake_piston) {
    m_rake_piston = std::move(rake_piston);
  }
}