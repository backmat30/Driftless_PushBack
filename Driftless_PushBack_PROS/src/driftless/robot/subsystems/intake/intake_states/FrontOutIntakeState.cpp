#include "driftless/robot/subsystems/intake/intake_states/FrontOutIntakeState.hpp"

namespace driftless::robot::subsystems::intake::intake_states {
void FrontOutIntakeState::update(StateMachineIntake& intake) {
  intake.setFrontMotorVoltage(-intake.getDesiredVoltage());
  intake.setFrontMotorCurrentLimit(2.5);

  intake.setIntermediaryMotorVoltage(0.0);
  intake.setIntermediaryMotorCurrentLimit(0.0);

  intake.setBackMotorVoltage(-intake.getDesiredVoltage());
  intake.setBackMotorCurrentLimit(1.25);

  intake.setVerticalMotorVoltage(-intake.getDesiredVoltage());
  intake.setVerticalMotorCurrentLimit(2.5);
}
}  // namespace driftless::robot::subsystems::intake::intake_states