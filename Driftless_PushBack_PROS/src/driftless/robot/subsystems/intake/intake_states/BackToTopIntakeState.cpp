#include "driftless/robot/subsystems/intake/intake_states/BackToTopIntakeState.hpp"

namespace driftless::robot::subsystems::intake::intake_states {
void BackToTopIntakeState::update(StateMachineIntake& intake) {
  intake.setFrontMotorVoltage(0.0);
  intake.setFrontMotorCurrentLimit(0.0);

  intake.setVerticalMotorVoltage(12.0);
  intake.setVerticalMotorCurrentLimit(2.5);

  intake.setIntermediaryMotorVoltage(12.0);
  intake.setIntermediaryMotorCurrentLimit(2.5);

  intake.setBackMotorVoltage(12.0);
  intake.setBackMotorCurrentLimit(2.5);
}
}  // namespace driftless::robot::subsystems::intake::intake_states