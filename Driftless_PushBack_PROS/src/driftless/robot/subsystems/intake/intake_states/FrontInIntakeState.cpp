#include "driftless/robot/subsystems/intake/intake_states/FrontInIntakeState.hpp"

namespace driftless::robot::subsystems::intake::intake_states {
void FrontInIntakeState::update(StateMachineIntake& intake) {
  if (!intake.isColorSortPaused()) {
    if (intake.hasOpposingBlock(StateMachineIntake::ColorSortLocation::MID)) {
      m_color_sort_active = true;
      m_latest_opposing_block_pos = intake.getFrontMotorPosition();
    }

    if (intake.hasOpposingBlock(StateMachineIntake::ColorSortLocation::BACK)) {
      m_outgoing_block_pos = intake.getBackMotorPosition();
      m_reject_block = true;
    }

    double color_sensor_distance{intake.getColorSensorPosition(
        StateMachineIntake::ColorSortLocation::MID)};

    if (m_color_sort_active &&
        (intake.getFrontMotorPosition() >
             m_latest_opposing_block_pos + color_sensor_distance * 4.0 ||
         intake.getFrontMotorPosition() <
             m_latest_opposing_block_pos - color_sensor_distance)) {
      m_latest_opposing_block_pos = -__DBL_MAX__;
      m_outgoing_block_pos = -__DBL_MAX__;
      m_color_sort_active = false;
      m_reject_block = false;
    }

    if (intake.getBackMotorPosition() > m_outgoing_block_pos + 2.5 ||
        intake.hasAllianceBlock(StateMachineIntake::ColorSortLocation::BACK)) {
      m_reject_block = false;
      m_outgoing_block_pos = -__DBL_MAX__;
    }

  } else {
    m_latest_opposing_block_pos = -__DBL_MAX__;
    m_color_sort_active = false;
    m_reject_block = false;
  }

  if (m_color_sort_active && !m_reject_block) {
    intake.setVerticalMotorVoltage(5.0);
    intake.setVerticalMotorCurrentLimit(2.5);

    intake.setIntermediaryMotorVoltage(4.0);
    intake.setIntermediaryMotorCurrentLimit(2.5);

    intake.setBackMotorVoltage(6.0);
    intake.setBackMotorCurrentLimit(1.0);
  } else if (m_reject_block) {
    intake.setBackMotorVoltage(6.0);
    intake.setBackMotorCurrentLimit(2.5);

    intake.setIntermediaryMotorVoltage(-12.0);
    intake.setIntermediaryMotorCurrentLimit(2.5);

    intake.setVerticalMotorVoltage(0.0);
    intake.setVerticalMotorCurrentLimit(2.5);
  } else {
    intake.setBackMotorVoltage(-intake.getDesiredVoltage() / 2.0);
    intake.setBackMotorCurrentLimit(1.25);

    intake.setIntermediaryMotorVoltage(intake.getDesiredVoltage());
    intake.setIntermediaryMotorCurrentLimit(1.5);

    intake.setVerticalMotorVoltage(12.0);
    intake.setVerticalMotorCurrentLimit(2.5);
  }

  intake.setFrontMotorVoltage(intake.getDesiredVoltage());
  intake.setFrontMotorCurrentLimit(2.5);

  // when we see block first, record position, set flag
  // if front travels far enough backwards or forwards since last block, stop
  // color sort Otherwise, if back sees block, record position if we are
  // actively color sorting, run the vertical motors slow
}
}  // namespace driftless::robot::subsystems::intake::intake_states