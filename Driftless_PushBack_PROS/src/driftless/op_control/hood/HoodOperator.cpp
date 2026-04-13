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

void HoodOperator::updateHoodToggle(EControllerDigital toggle_high_goal,
                                    EControllerDigital toggle_low_goal,
                                    EControllerDigital toggle_gate,
                                    EControllerDigital toggle_descore) {
  bool toggle_high_goal_pressed{m_controller->getNewDigital(toggle_high_goal)};
  bool toggle_low_goal_pressed{m_controller->getNewDigital(toggle_low_goal)};
  bool toggle_gate_pressed{m_controller->getNewDigital(toggle_gate)};
  bool activate_descore_pressed{m_controller->getNewDigital(toggle_descore)};

  bool is_hood_raised{*static_cast<bool*>(
      m_robot->getState(robot::subsystems::ESubsystem::HOOD,
                        robot::subsystems::ESubsystemState::HOOD_IS_RAISED))};
  bool is_hood_bumped{*static_cast<bool*>(
      m_robot->getState(robot::subsystems::ESubsystem::HOOD,
                        robot::subsystems::ESubsystemState::HOOD_IS_BUMPED))};
  bool is_descore_mid{
      (*static_cast<int*>(m_robot->getState(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemState::HOOD_GET_DESCORE_HEIGHT))) == 1};

  if (toggle_high_goal_pressed) {
    if (is_hood_raised) {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_CLOSE_GATE);
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_RETRACT_DESCORE);
    } else {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_OPEN_GATE);
    }
    m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                         robot::subsystems::ESubsystemCommand::HOOD_BUMP_DOWN);
    toggleHoodHeight();
  } else if (toggle_low_goal_pressed) {
    if (is_hood_bumped) {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_CLOSE_GATE);
    } else {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_OPEN_GATE);
    }
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_RETRACT_DESCORE);
    m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                         robot::subsystems::ESubsystemCommand::HOOD_LOWER);
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_TOGGLE_BUMP);
  } else if (toggle_gate_pressed) {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_TOGGLE_GATE);
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_RETRACT_DESCORE);
  }

  if (activate_descore_pressed && !is_descore_mid) {
    if (!is_hood_raised && !is_hood_bumped) {
      m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                           robot::subsystems::ESubsystemCommand::HOOD_RAISE);
    }
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_EXTEND_DESCORE_HALF);
    m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                         robot::subsystems::ESubsystemCommand::HOOD_CLOSE_GATE);
  } else if (activate_descore_pressed && is_descore_mid) {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_EXTEND_DESCORE);
  }
}

void HoodOperator::updateHoodHold(EControllerDigital toggle_high_goal,
                                  EControllerDigital toggle_low_goal,
                                  EControllerDigital toggle_gate,
                                  EControllerDigital hold_descore) {
  bool toggle_high_goal_pressed{m_controller->getNewDigital(toggle_high_goal)};
  bool toggle_low_goal_pressed{m_controller->getNewDigital(toggle_low_goal)};
  bool toggle_gate_pressed{m_controller->getNewDigital(toggle_gate)};
  bool activate_descore_pressed{m_controller->getDigital(hold_descore)};

  bool is_hood_raised{*static_cast<bool*>(
      m_robot->getState(robot::subsystems::ESubsystem::HOOD,
                        robot::subsystems::ESubsystemState::HOOD_IS_RAISED))};
  bool is_hood_bumped{*static_cast<bool*>(
      m_robot->getState(robot::subsystems::ESubsystem::HOOD,
                        robot::subsystems::ESubsystemState::HOOD_IS_BUMPED))};
  bool is_descore_mid{
      (*static_cast<int*>(m_robot->getState(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemState::HOOD_GET_DESCORE_HEIGHT))) == 1};

  if (toggle_high_goal_pressed) {
    if (is_hood_raised) {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_CLOSE_GATE);
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_RETRACT_DESCORE);
    } else {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_OPEN_GATE);
    }
    m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                         robot::subsystems::ESubsystemCommand::HOOD_BUMP_DOWN);
    toggleHoodHeight();
  } else if (toggle_low_goal_pressed) {
    if (is_hood_bumped) {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_CLOSE_GATE);
    } else {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_OPEN_GATE);
    }
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_RETRACT_DESCORE);
    m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                         robot::subsystems::ESubsystemCommand::HOOD_LOWER);
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_TOGGLE_BUMP);
  } else if (toggle_gate_pressed) {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_TOGGLE_GATE);
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_RETRACT_DESCORE);
  }

  if (activate_descore_pressed) {
    if (!is_hood_raised && !is_hood_bumped) {
      m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                           robot::subsystems::ESubsystemCommand::HOOD_RAISE);
    }
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_EXTEND_DESCORE_HALF);
    m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                         robot::subsystems::ESubsystemCommand::HOOD_CLOSE_GATE);
  } else if (is_descore_mid) {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_EXTEND_DESCORE);
  }
}

void HoodOperator::updateHoodRollers(EControllerDigital spin_forwards_front,
                                     EControllerDigital spin_forwards_back,
                                     EControllerDigital spin_backwards) {
  bool spin_forwards_pressed{m_controller->getDigital(spin_forwards_front) ||
                             m_controller->getDigital(spin_forwards_back)};
  bool spin_backwards_pressed{m_controller->getDigital(spin_backwards)};

  bool is_hood_open{*static_cast<bool*>(
      m_robot->getState(robot::subsystems::ESubsystem::HOOD,
                        robot::subsystems::ESubsystemState::HOOD_IS_OPEN))};

  if (spin_forwards_pressed) {
    setMotorVoltage(12.0);
    if (!is_hood_open) {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_SET_CURRENT_LIMIT, 1.35);
    } else {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_SET_CURRENT_LIMIT, 2.5);
    }
  } else {
    if (spin_backwards_pressed) {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::HOOD,
          robot::subsystems::ESubsystemCommand::HOOD_SET_CURRENT_LIMIT, 2.5);
      setMotorVoltage(-12.0);
    } else {
      setMotorVoltage(0.0);
    }
  }
}

HoodOperator::HoodOperator(const std::shared_ptr<io::IController>& controller,
                           const std::shared_ptr<robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void HoodOperator::update(const std::unique_ptr<profiles::IProfile>& profile) {
  EHoodControlMode control_mode{static_cast<EHoodControlMode>(
      profile->getControlMode(EControlType::HOOD))};

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

  updateHoodRollers(spin_forwards_front, spin_forwards_back, spin_backwards);

  switch (control_mode) {
    case EHoodControlMode::DESCORE_TOGGLE:
      updateHoodToggle(toggle_height, toggle_bump, toggle_gate, toggle_descore);
      break;
    case EHoodControlMode::DESCORE_HOLD:
      updateHoodHold(toggle_height, toggle_bump, toggle_gate, toggle_descore);
      break;
  }
}
}  // namespace driftless::op_control::hood