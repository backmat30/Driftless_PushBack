#include "driftless/robot/subsystems/intake/DirectIntakeBuilder.hpp"

namespace driftless::robot::subsystems::intake {
DirectIntakeBuilder* DirectIntakeBuilder::withMotor(
    std::unique_ptr<io::IMotor>& motor) {
  m_motors.addMotor(motor);
  return this;
}

DirectIntakeBuilder* DirectIntakeBuilder::withPiston(
    std::unique_ptr<io::IPiston>& piston) {
  m_pistons.addPiston(piston);
  return this;
}

std::unique_ptr<IIntake> DirectIntakeBuilder::build() {
  std::unique_ptr<DirectIntake> direct_intake{std::make_unique<DirectIntake>()};
  direct_intake->setMotors(m_motors);
  direct_intake->setPistons(m_pistons);

  return direct_intake;
}
}  // namespace driftless::robot::subsystems::intake