#include "driftless/robot/subsystems/hood/DirectHoodBuilder.hpp"

namespace driftless::robot::subsystems::hood {
DirectHoodBuilder* DirectHoodBuilder::withMotor(
    std::unique_ptr<io::IMotor>& motor) {
  m_motors.addMotor(motor);
  return this;
}

DirectHoodBuilder* DirectHoodBuilder::withHeightPiston(
    std::unique_ptr<io::IPiston>& piston) {
  m_height_pistons.addPiston(piston);
  return this;
}

DirectHoodBuilder* DirectHoodBuilder::withGatePiston(
    std::unique_ptr<io::IPiston>& piston) {
  m_gate_pistons.addPiston(piston);
  return this;
}

DirectHoodBuilder* DirectHoodBuilder::withLowerDescorePiston(
    std::unique_ptr<io::IPiston>& piston) {
  m_lower_descore_pistons.addPiston(piston);
  return this;
}

DirectHoodBuilder* DirectHoodBuilder::withUpperDescorePiston(
    std::unique_ptr<io::IPiston>& piston) {
  m_upper_descore_pistons.addPiston(piston);
  return this;
}

DirectHoodBuilder* DirectHoodBuilder::withBumpPiston(
    std::unique_ptr<io::IPiston>& piston) {
  m_bump_pistons.addPiston(piston);
  return this;
}

std::unique_ptr<IHood> DirectHoodBuilder::build() {
  std::unique_ptr<DirectHood> direct_hood{std::make_unique<DirectHood>()};

  direct_hood->setMotors(m_motors);
  direct_hood->setHeightPistons(m_height_pistons);
  direct_hood->setGatePistons(m_gate_pistons);
  direct_hood->setLowerDescorePistons(m_lower_descore_pistons);
  direct_hood->setUpperDescorePistons(m_upper_descore_pistons);
  direct_hood->setBumpPistons(m_bump_pistons);

  return direct_hood;
}
}  // namespace driftless::robot::subsystems::hood
