#include "driftless/robot/subsystems/intake/DirectIntakeBuilder.hpp"

namespace driftless::robot::subsystems::intake {
DirectIntakeBuilder* DirectIntakeBuilder::withFrontMotor(
    std::unique_ptr<io::IMotor>& motor) {
  m_front_motors.addMotor(motor);
  return this;
}

DirectIntakeBuilder* DirectIntakeBuilder::withIntermediaryMotor(std::unique_ptr<io::IMotor>& motor) {
  m_intermediary_motors.addMotor(motor);
  return this;
}

DirectIntakeBuilder* DirectIntakeBuilder::withBackMotor(std::unique_ptr<io::IMotor>& motor) {
  m_back_motors.addMotor(motor);
  return this;
}

DirectIntakeBuilder* DirectIntakeBuilder::withVerticalMotor(std::unique_ptr<io::IMotor>& motor) {
  m_vertical_motors.addMotor(motor);
  return this;
}

DirectIntakeBuilder* DirectIntakeBuilder::withBackPiston(
    std::unique_ptr<io::IPiston>& piston) {
  m_back_pistons.addPiston(piston);
  return this;
}

std::unique_ptr<IIntake> DirectIntakeBuilder::build() {
  std::unique_ptr<DirectIntake> direct_intake{std::make_unique<DirectIntake>()};
  direct_intake->setFrontMotors(m_front_motors);
  direct_intake->setIntermediaryMotors(m_intermediary_motors);
  direct_intake->setBackMotors(m_back_motors);
  direct_intake->setVerticalMotors(m_vertical_motors);
  direct_intake->setBackPistons(m_back_pistons);

  return direct_intake;
}
}  // namespace driftless::robot::subsystems::intake