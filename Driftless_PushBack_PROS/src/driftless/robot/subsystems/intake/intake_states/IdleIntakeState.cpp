#include "driftless/robot/subsystems/intake/intake_states/IdleIntakeState.hpp"

namespace driftless::robot::subsystems::intake::intake_states {
void IdleIntakeState::update(StateMachineIntake& intake) {
  intake.setFrontMotorVoltage(0);
  intake.setFrontMotorCurrentLimit(0.0);

  intake.setIntermediaryMotorVoltage(0);
  intake.setIntermediaryMotorCurrentLimit(0.0);

  intake.setBackMotorVoltage(0);
  intake.setBackMotorCurrentLimit(0.0);

  intake.setVerticalMotorVoltage(0);
  intake.setVerticalMotorCurrentLimit(0.0);
}
}