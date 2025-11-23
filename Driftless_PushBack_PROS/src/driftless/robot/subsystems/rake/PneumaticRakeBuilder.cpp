#include "driftless/robot/subsystems/rake/PneumaticRakeBuilder.hpp"

namespace driftless::robot::subsystems::rake {
  PneumaticRakeBuilder* PneumaticRakeBuilder::withRakePiston(std::unique_ptr<io::IPiston>& rake_piston) {
    m_rake_piston = std::move(rake_piston);
    return this;
  }

  std::unique_ptr<PneumaticRake> PneumaticRakeBuilder::build() {
    std::unique_ptr<PneumaticRake> pneumatic_rake{std::make_unique<PneumaticRake>()};

    pneumatic_rake->setRakePiston(m_rake_piston);

    return pneumatic_rake;
  }
}