#include "driftless/op_control/brake/BrakeOperator.hpp"

namespace driftless::op_control::brake {
  void BrakeOperator::updateToggle(EControllerDigital brake_toggle) {
    bool brake_toggle_pressed{m_controller->getNewDigital(brake_toggle)};

    bool is_brake_engaged{*static_cast<bool*>(m_robot->getState(robot::subsystems::ESubsystem::BRAKE, robot::subsystems::ESubsystemState::BRAKE_IS_DEPLOYED))};

    if(brake_toggle_pressed) {
      if(is_brake_engaged) {
        m_robot->sendCommand(robot::subsystems::ESubsystem::BRAKE, robot::subsystems::ESubsystemCommand::BRAKE_RETRACT);
      } else {
        m_robot->sendCommand(robot::subsystems::ESubsystem::BRAKE, robot::subsystems::ESubsystemCommand::BRAKE_DEPLOY);
      }
    }
  }

  BrakeOperator::BrakeOperator(std::shared_ptr<io::IController>& controller, std::shared_ptr<robot::Robot>& robot) : m_robot{robot}, m_controller{controller} {}

  void BrakeOperator::update(std::shared_ptr<profiles::IProfile>& profile) {
    EControllerDigital brake_toggle{profile->getDigitalControlMapping(EControl::BRAKE_TOGGLE)};

    updateToggle(brake_toggle);
  }
}