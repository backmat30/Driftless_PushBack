#include "driftless/robot/subsystems/intake/intake_states/FrontInIntakeState.hpp"

namespace driftless::robot::subsystems::intake::intake_states {
void FrontInIntakeState::update(StateMachineIntake& intake) {
  if (!intake.m_color_sort_paused) {
    if (intake.hasOpposingBlock()) {
      m_latest_opposing_block_pos = intake.m_front_motors.getPosition();
    }
    if (intake.m_front_motors.getPosition() >
            m_latest_opposing_block_pos +
                intake.COLOR_SORT_DISTANCE_TO_END * 2.0 ||
        intake.m_front_motors.getPosition() <
            m_latest_opposing_block_pos - intake.COLOR_SORT_DISTANCE_TO_END) {
      m_latest_opposing_block_pos = -__DBL_MAX__;
    } else if (intake.m_front_motors.getPosition() <
                   m_latest_opposing_block_pos +
                       intake.COLOR_SORT_DISTANCE_TO_END &&
               intake.m_running_forward) {
      intake.m_back_motors.setVoltage(-12.0);
      intake.m_back_motors.setCurrentLimit(2.5);
      intake.m_intermediary_motors.setVoltage(-12.0);
      intake.m_intermediary_motors.setCurrentLimit(2.5);
    }
  }
}
}  // namespace driftless::robot::subsystems::intake::intake_states