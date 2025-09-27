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
                                    EControllerDigital toggle_gate) {
  bool spin_forwards_pressed{m_controller->getDigital(spin_forwards_front) || m_controller->getDigital(spin_forwards_back)};
  bool spin_backwards_pressed{m_controller->getDigital(spin_backwards)};
  bool toggle_height_pressed{m_controller->getNewDigital(toggle_height)};
  bool toggle_gate_pressed{m_controller->getNewDigital(toggle_gate)};

  if (spin_forwards_pressed) {
    setMotorVoltage(12.0);
  } else if (spin_backwards_pressed) {
    setMotorVoltage(-12.0);
  } else {
    setMotorVoltage(0.0);
  }

  if (toggle_height_pressed) {
    toggleHoodHeight();
  }

  if (toggle_gate_pressed) {
    toggleHoodGate();
  }
}

HoodOperator::HoodOperator(const std::shared_ptr<io::IController>& controller,
                           const std::shared_ptr<robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void HoodOperator::update(const std::unique_ptr<profiles::IProfile>& profile) {
  EControllerDigital spin_forwards_front{
    profile->getDigitalControlMapping(op_control::EControl::INTAKE_FRONT_RUN_IN)
  };
  EControllerDigital spin_forwards_back{
    profile->getDigitalControlMapping(op_control::EControl::INTAKE_BACK_RUN_IN)
  };
  EControllerDigital spin_backwards{
    profile->getDigitalControlMapping(op_control::EControl::INTAKE_FRONT_RUN_OUT)
  };
  EControllerDigital toggle_height{
    profile->getDigitalControlMapping(op_control::EControl::HOOD_TOGGLE_RAISED)
  };
  EControllerDigital toggle_gate{
    profile->getDigitalControlMapping(op_control::EControl::HOOD_TOGGLE_GATE)
  };

  updateHoodToggle(spin_forwards_front, spin_forwards_back, spin_backwards,
                   toggle_height, toggle_gate);
}
}  // namespace driftless::op_control::hood