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

  double front_voltage{};
  double intermediary_voltage{};
  double back_voltage{};
  double vertical_voltage{};

  if (run_back_intake_in) {
    back_voltage = -12.0;
    intermediary_voltage = 12.0;
    vertical_voltage = 12.0;

  } else if (run_front_intake_in) {
    front_voltage = 12.0;
    intermediary_voltage = 12.0;
    back_voltage = -6.0;
    vertical_voltage = 12.0;
  
  } else if (run_front_intake_out) {
    front_voltage = -12.0;
    back_voltage = -12.0;
    intermediary_voltage = -12.0;
    vertical_voltage = -12.0;
  }

  m_robot->sendCommand(
      robot::subsystems::ESubsystem::INTAKE,
      robot::subsystems::ESubsystemCommand::INTAKE_SET_FRONT_VOLTAGE, front_voltage);
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::INTAKE,
      robot::subsystems::ESubsystemCommand::INTAKE_SET_INTERMEDIARY_VOLTAGE,
      intermediary_voltage);
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::INTAKE,
      robot::subsystems::ESubsystemCommand::INTAKE_SET_BACK_VOLTAGE,
      back_voltage);
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::INTAKE,
      robot::subsystems::ESubsystemCommand::INTAKE_SET_VERTICAL_VOLTAGE,
      vertical_voltage);

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

IntakeOperator::IntakeOperator(
    const std::shared_ptr<io::IController>& controller,
    const std::shared_ptr<robot::Robot>& robot)
    : m_controller{controller}, m_robot{robot} {}

void IntakeOperator::update(
    const std::unique_ptr<profiles::IProfile>& profile) {
  EControllerDigital front_intake_in{
      profile->getDigitalControlMapping(EControl::INTAKE_FRONT_RUN_IN)};
  EControllerDigital front_intake_out{
      profile->getDigitalControlMapping(EControl::INTAKE_FRONT_RUN_OUT)};
  EControllerDigital back_intake_in{
      profile->getDigitalControlMapping(EControl::INTAKE_BACK_RUN_IN)};
  EControllerDigital back_arms_toggle{
      profile->getDigitalControlMapping(EControl::INTAKE_BACK_TOGGLE_ARMS)};

  updateIntakeSplit(front_intake_in, front_intake_out, back_intake_in,
                    back_arms_toggle);
}
}  // namespace driftless::op_control::intake