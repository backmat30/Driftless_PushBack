#include "driftless/robot/subsystems/brake/PneumaticBrakeBuilder.hpp"

namespace driftless::robot::subsystems::brake {
  PneumaticBrakeBuilder* PneumaticBrakeBuilder::withBrakePiston(std::unique_ptr<io::IPiston>& brake_piston) {
    m_brake_piston = std::move(brake_piston);
    return this;
  }

  std::unique_ptr<PneumaticBrake> PneumaticBrakeBuilder::build() {
    std::unique_ptr<PneumaticBrake> pneumatic_brake{std::make_unique<PneumaticBrake>()};

    pneumatic_brake->setBrakePiston(m_brake_piston);

    return pneumatic_brake;
  }
}