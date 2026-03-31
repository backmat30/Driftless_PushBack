#include "driftless/robot/subsystems/intake/intake_states/FrontInIntakeState.hpp"

namespace driftless::robot::subsystems::intake::intake_states {
void FrontInIntakeState::update(StateMachineIntake& intake) {
  if (!intake.isColorSortPaused()) {
    if (intake.hasOpposingBlock()) {
      m_latest_opposing_block_pos = intake.getFrontMotorPosition();
    }
    if (intake.getFrontMotorPosition() >
            m_latest_opposing_block_pos +
                intake.getColorSensorPosition() * 2.0 ||
        intake.getFrontMotorPosition() <
            m_latest_opposing_block_pos - intake.getColorSensorPosition()) {
      m_latest_opposing_block_pos = -__DBL_MAX__;
    } else if (intake.getFrontMotorPosition() <
               m_latest_opposing_block_pos + intake.getColorSensorPosition()) {
      intake.setBackMotorVoltage(-12.0);
      intake.setBackMotorCurrentLimit(2.5);
      intake.setIntermediaryMotorVoltage(-12.0);
      intake.setIntermediaryMotorCurrentLimit(2.5);
    }
  }

  if (intake.getFrontMotorPosition() >
      m_latest_opposing_block_pos + intake.getColorSensorPosition()) {
    intake.setBackMotorVoltage(-intake.getDesiredVoltage() / 2.0);
    intake.setBackMotorCurrentLimit(1.25);
    intake.setIntermediaryMotorVoltage(intake.getDesiredVoltage());
    intake.setIntermediaryMotorCurrentLimit(1.5);
  }

  intake.setFrontMotorVoltage(intake.getDesiredVoltage());
  intake.setFrontMotorCurrentLimit(2.5);
  intake.setVerticalMotorVoltage(intake.getDesiredVoltage());
  intake.setVerticalMotorCurrentLimit(2.5);
}
}  // namespace driftless::robot::subsystems::intake::intake_states