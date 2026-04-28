#include "driftless/op_control/intake/IntakeOperator.hpp"

namespace driftless::op_control::intake {
void IntakeOperator::updateIntakeSplit(EControllerDigital front_intake_in,
                                       EControllerDigital front_intake_out,
                                       EControllerDigital back_intake_in,
                                       EControllerDigital back_arms_toggle) {
  bool run_front_intake_in{m_controller->getDigital(front_intake_in)};
  bool run_front_intake_out{m_controller->getDigital(front_intake_out)};
  bool run_back_intake_in{m_controller->getDigital(back_intake_in)};
  bool toggle_back_arms{m_controller->getNewDigital(back_arms_toggle)};

  if (run_back_intake_in) {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::INTAKE,
        robot::subsystems::ESubsystemCommand::INTAKE_BACK_TO_HOOD);

  } else if (run_front_intake_in) {
    m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                         robot::subsystems::ESubsystemCommand::INTAKE_FRONT,
                         12.0);

  } else if (run_front_intake_out) {
    m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                         robot::subsystems::ESubsystemCommand::INTAKE_FRONT,
                         -12.0);
  } else {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::INTAKE,
        robot::subsystems::ESubsystemCommand::INTAKE_STOP_MOTION);
  }

  if (toggle_back_arms) {
    bool is_intake_deployed{*static_cast<bool*>(m_robot->getState(
        robot::subsystems::ESubsystem::INTAKE,
        robot::subsystems::ESubsystemState::INTAKE_ARE_ARMS_DEPLOYED))};

    if (is_intake_deployed) {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::INTAKE,
          robot::subsystems::ESubsystemCommand::INTAKE_RETRACT_ARMS);
    } else {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::INTAKE,
          robot::subsystems::ESubsystemCommand::INTAKE_DEPLOY_ARMS);
    }
  }
}

void IntakeOperator::updateIntakeSmartSplit(
    EControllerDigital front_intake_in, EControllerDigital front_intake_out,
    EControllerDigital back_intake_in,
    EControllerDigital back_intake_to_bottom) {
  bool run_front_intake_in{m_controller->getDigital(front_intake_in)};
  bool run_front_intake_out{m_controller->getDigital(front_intake_out)};
  bool run_back_intake_in{m_controller->getDigital(back_intake_in)};
  bool run_back_intake_to_back{
      m_controller->getNewDigital(back_intake_to_bottom)};
  bool stop_back_intake{!m_controller->getDigital(back_intake_to_bottom)};

  if (run_back_intake_in) {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::INTAKE,
        robot::subsystems::ESubsystemCommand::INTAKE_BACK_TO_HOOD);

    m_robot->sendCommand(
        robot::subsystems::ESubsystem::INTAKE,
        robot::subsystems::ESubsystemCommand::INTAKE_DEPLOY_ARMS);

  } else if (run_back_intake_to_back) {
    m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                         robot::subsystems::ESubsystemCommand::INTAKE_BACK_IN);

    m_robot->sendCommand(
        robot::subsystems::ESubsystem::INTAKE,
        robot::subsystems::ESubsystemCommand::INTAKE_DEPLOY_ARMS);

  } else if (stop_back_intake) {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::INTAKE,
        robot::subsystems::ESubsystemCommand::INTAKE_RETRACT_ARMS);

    if (run_front_intake_in) {
      m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                           robot::subsystems::ESubsystemCommand::INTAKE_FRONT,
                           12.0);

    } else if (run_front_intake_out) {
      m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                           robot::subsystems::ESubsystemCommand::INTAKE_FRONT,
                           -12.0);
    } else {
      m_robot->sendCommand(
          robot::subsystems::ESubsystem::INTAKE,
          robot::subsystems::ESubsystemCommand::INTAKE_STOP_MOTION);
    }
  }
}

IntakeOperator::IntakeOperator(
    const std::shared_ptr<io::IController>& controller,
    const std::shared_ptr<robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void IntakeOperator::update(
    const std::unique_ptr<profiles::IProfile>& profile) {
  EIntakeControlMode control_mode{static_cast<EIntakeControlMode>(
      profile->getControlMode(EControlType::INTAKE))};

  EControllerDigital front_intake_in{
      profile->getDigitalControlMapping(EControl::INTAKE_FRONT_RUN_IN)};
  EControllerDigital front_intake_out{
      profile->getDigitalControlMapping(EControl::INTAKE_FRONT_RUN_OUT)};
  EControllerDigital back_intake_in{
      profile->getDigitalControlMapping(EControl::INTAKE_BACK_RUN_IN)};
  EControllerDigital back_arms_toggle{
      profile->getDigitalControlMapping(EControl::INTAKE_BACK_TOGGLE_ARMS)};
  EControllerDigital back_intake_to_bottom{
      profile->getDigitalControlMapping(EControl::INTAKE_BACK_TO_BOTTOM)};

  switch (control_mode) {
    case EIntakeControlMode::SPLIT:
      updateIntakeSplit(front_intake_in, front_intake_out, back_intake_in,
                        back_arms_toggle);
      break;
    case EIntakeControlMode::SMART_SPLIT:
      updateIntakeSmartSplit(front_intake_in, front_intake_out, back_intake_in,
                             back_intake_to_bottom);
      break;
  }
}
}  // namespace driftless::op_control::intake