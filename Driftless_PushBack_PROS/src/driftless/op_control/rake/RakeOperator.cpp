#include "driftless/op_control/rake/RakeOperator.hpp"

namespace driftless::op_control::rake {
void RakeOperator::updateToggle(EControllerDigital rake_toggle) {
  bool rake_toggle_pressed{m_controller->getNewDigital(rake_toggle)};

  bool is_rake_engaged{*static_cast<bool*>(
      m_robot->getState(robot::subsystems::ESubsystem::RAKE,
                        robot::subsystems::ESubsystemState::RAKE_IS_DEPLOYED))};

  if (rake_toggle_pressed) {
    if (is_rake_engaged) {
      m_robot->sendCommand(robot::subsystems::ESubsystem::RAKE,
                           robot::subsystems::ESubsystemCommand::RAKE_RETRACT);
    } else {
      m_robot->sendCommand(robot::subsystems::ESubsystem::RAKE,
                           robot::subsystems::ESubsystemCommand::RAKE_DEPLOY);
    }
  }
}

RakeOperator::RakeOperator(std::shared_ptr<io::IController>& controller,
                           std::shared_ptr<robot::Robot>& robot)
    : m_robot{robot}, m_controller{controller} {}

void RakeOperator::update(std::unique_ptr<profiles::IProfile>& profile) {
  EControllerDigital rake_toggle{
      profile->getDigitalControlMapping(EControl::RAKE_TOGGLE)};

  updateToggle(rake_toggle);
}
}  // namespace driftless::op_control::rake