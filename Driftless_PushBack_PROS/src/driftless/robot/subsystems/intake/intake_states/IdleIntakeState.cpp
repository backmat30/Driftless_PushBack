#include "driftless/robot/subsystems/intake/intake_states/IdleIntakeState.hpp"

namespace driftless::robot::subsystems::intake::intake_states {
void IdleIntakeState::update(StateMachineIntake& intake) {
  intake.m_front_motors.setVoltage(0);
  intake.m_intermediary_motors.setVoltage(0);
  intake.m_back_motors.setVoltage(0);
  intake.m_vertical_motors.setVoltage(0);
}
}