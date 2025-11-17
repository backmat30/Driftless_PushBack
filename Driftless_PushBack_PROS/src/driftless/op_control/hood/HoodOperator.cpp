#include "driftless/op_control/hood/HoodOperator.hpp"

namespace driftless::op_control::hood {
void HoodOperator::setMotorVoltage(double voltage) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                       robot::subsystems::ESubsystemCommand::HOOD_SET_VOLTAGE,
                       voltage);
}

void HoodOperator::toggleHoodHeight() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::HOOD,
      robot::subsystems::ESubsystemCommand::HOOD_TOGGLE_RAISED);
}

void HoodOperator::toggleHoodGate() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                       robot::subsystems::ESubsystemCommand::HOOD_TOGGLE_GATE);
}

void HoodOperator::updateHoodToggle(EControllerDigital spin_forwards_front,
                                    EControllerDigital spin_forwards_back,
                                    EControllerDigital spin_backwards,
                                    EControllerDigital toggle_height,
                                    EControllerDigital toggle_gate,
                                    EControllerDigital toggle_descore,
                                    EControllerDigital toggle_bump) {
  bool spin_forwards_pressed{m_controller->getDigital(spin_forwards_front) ||
                             m_controller->getDigital(spin_forwards_back)};
  bool spin_backwards_pressed{m_controller->getDigital(spin_backwards)};
  bool toggle_height_pressed{m_controller->getNewDigital(toggle_height)};
  bool toggle_gate_pressed{m_controller->getNewDigital(toggle_gate)};
  bool toggle_descore_pressed{m_controller->getNewDigital(toggle_descore)};
  bool toggle_bump_pressed{m_controller->getNewDigital(toggle_bump)};

  bool is_hood_raised{*static_cast<bool*>(
        m_robot->getState(robot::subsystems::ESubsystem::HOOD,
                          robot::subsystems::ESubsystemState::HOOD_IS_RAISED))};

  if (spin_forwards_pressed) {
    setMotorVoltage(12.0);
  } else if (spin_backwards_pressed) {
    setMotorVoltage(-12.0);
  } else {
    setMotorVoltage(0.0);
  }

  if (toggle_height_pressed) {
    toggleHoodHeight();
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_RETRACT_DESCORE);
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_BUMP_DOWN);
  }

  if (toggle_gate_pressed) {
    toggleHoodGate();
  }

  if (toggle_descore_pressed) {
    if (is_hood_raised) {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_TOGGLE_DESCORE);
    }
  }

  if (toggle_bump_pressed) {
    if (!is_hood_raised) {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_TOGGLE_BUMP);
    }
  }
}

HoodOperator::HoodOperator(const std::shared_ptr<io::IController>& controller,
                           const std::shared_ptr<robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void HoodOperator::update(const std::unique_ptr<profiles::IProfile>& profile) {
  EControllerDigital spin_forwards_front{profile->getDigitalControlMapping(
      op_control::EControl::INTAKE_FRONT_RUN_IN)};
  EControllerDigital spin_forwards_back{profile->getDigitalControlMapping(
      op_control::EControl::INTAKE_BACK_RUN_IN)};
  EControllerDigital spin_backwards{profile->getDigitalControlMapping(
      op_control::EControl::INTAKE_FRONT_RUN_OUT)};
  EControllerDigital toggle_height{profile->getDigitalControlMapping(
      op_control::EControl::HOOD_TOGGLE_RAISED)};
  EControllerDigital toggle_gate{profile->getDigitalControlMapping(
      op_control::EControl::HOOD_TOGGLE_GATE)};
  EControllerDigital toggle_descore{profile->getDigitalControlMapping(
      op_control::EControl::HOOD_TOGGLE_DESCORE)};
  EControllerDigital toggle_bump{profile->getDigitalControlMapping(
      op_control::EControl::HOOD_TOGGLE_BUMP)};

  updateHoodToggle(spin_forwards_front, spin_forwards_back, spin_backwards,
                   toggle_height, toggle_gate, toggle_descore, toggle_bump);
}
}  // namespace driftless::op_control::hood