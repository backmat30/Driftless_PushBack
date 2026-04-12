#include "driftless/robot/subsystems/intake/intake_states/BackToBottomIntakeState.hpp"

#include "pros/screen.hpp"
namespace driftless::robot::subsystems::intake::intake_states {
void BackToBottomIntakeState::update(StateMachineIntake& intake) {
  // detect first block and record position
  if (intake.hasAllianceBlock() && !m_has_first_matchloader_block) {
    m_first_matchloader_block_pos = intake.getFrontMotorPosition();
    m_has_first_matchloader_block = true;
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5, "Has block 1");
    // if we have the first block, are not yet ready for the second block, and
    // there is either no block visible or we have moved the front intake far
    // enough, declare that we are ready for block 2
  } else if (m_has_first_matchloader_block &&
             !m_ready_for_second_matchloader_block &&
             intake.getFrontMotorPosition() <
                 m_first_matchloader_block_pos - 3.0) {
    m_ready_for_second_matchloader_block = true;
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 6, "Ready for block 2");
    // if we are ready for a second block but do not have a second block,
    // check if we can see a second block and update flag
  } else if (m_ready_for_second_matchloader_block &&
             !m_has_second_matchloader_block && intake.hasBlock()) {
    m_has_second_matchloader_block = true;
    m_second_matchloader_block_pos = intake.getFrontMotorPosition();
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 7, "has block 2");
    // wait for the front intake to go far enough, then update the flag to
    // direct blocks to the hood
  } else if (m_has_second_matchloader_block &&
             intake.getFrontMotorPosition() <
                 m_second_matchloader_block_pos - 3.0) {
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 8, "Bottom full");
    m_has_first_matchloader_block = false;
    m_ready_for_second_matchloader_block = false;
    m_has_second_matchloader_block = false;
    m_first_matchloader_block_pos = -__DBL_MAX__;
    m_second_matchloader_block_pos = -__DBL_MAX__;

    intake.setCurrentState(intake_states::EIntakeStates::BACK_IN_TOP);
  }

  if (!m_has_first_matchloader_block ||
      (m_has_first_matchloader_block &&
       !m_ready_for_second_matchloader_block)) {
    intake.setFrontMotorVoltage(-6.0);
    intake.setFrontMotorCurrentLimit(2.5);

    intake.setVerticalMotorVoltage(-12.0);
    intake.setVerticalMotorCurrentLimit(2.5);
  } else if (m_has_second_matchloader_block) {
    intake.setFrontMotorVoltage(-6.0);
    intake.setFrontMotorCurrentLimit(2.5);

    intake.setVerticalMotorVoltage(-10.0);
    intake.setVerticalMotorCurrentLimit(1.0);
  } else {
    intake.setFrontMotorVoltage(0.0);
    intake.setFrontMotorCurrentLimit(0.0);

    intake.setVerticalMotorVoltage(-6.0);
    intake.setVerticalMotorCurrentLimit(1.0);
  }

  intake.setIntermediaryMotorVoltage(12.0);
  intake.setIntermediaryMotorCurrentLimit(2.5);

  intake.setBackMotorVoltage(12.0);
  intake.setBackMotorCurrentLimit(1.0);
}
}  // namespace driftless::robot::subsystems::intake::intake_states