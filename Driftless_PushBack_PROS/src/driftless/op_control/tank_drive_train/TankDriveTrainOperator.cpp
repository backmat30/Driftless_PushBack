#include "driftless/op_control/tank_drive_train/TankDriveTrainOperator.hpp"
namespace driftless {
namespace op_control {
namespace tank_drive_train {
void TankDriveTrainOperator::updateDriveVoltage(double left_voltage,
                                                double right_voltage) {
  if (m_robot) {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::DRIVETRAIN,
        robot::subsystems::ESubsystemCommand::DRIVETRAIN_SET_VOLTAGE,
        left_voltage, right_voltage);
  }
}

void TankDriveTrainOperator::updateTank() {
  double left_voltage{
      m_controller->getAnalog(EControllerAnalog::JOYSTICK_LEFT_Y) *
      VOLTAGE_CONVERSION};
  double right_voltage{
      m_controller->getAnalog(EControllerAnalog::JOYSTICK_RIGHT_Y) *
      VOLTAGE_CONVERSION};

  updateDriveVoltage(left_voltage, right_voltage);
}

void TankDriveTrainOperator::updateArcade(EControllerAnalog linear,
                                          EControllerAnalog turn) {
  double linear_value{m_controller->getAnalog(linear)};
  double turn_value{m_controller->getAnalog(turn)};

  double left_voltage{(linear_value + turn_value) * VOLTAGE_CONVERSION};
  double right_voltage{(linear_value - turn_value) * VOLTAGE_CONVERSION};

  updateDriveVoltage(left_voltage, right_voltage);
}

TankDriveTrainOperator::TankDriveTrainOperator(
    const std::shared_ptr<io::IController> &controller,
    const std::shared_ptr<robot::Robot> &robot)
    : m_controller{controller}, m_robot{robot} {}

void TankDriveTrainOperator::setDriveVoltage(
    std::unique_ptr<driftless::profiles::IProfile> &profile) {
  if (!m_controller) {
    updateDriveVoltage(0, 0);
    return;
  }

  EControllerAnalog linear{
      profile->getAnalogControlMapping(EControl::TANK_DRIVE_ARCADE_LINEAR)};
  EControllerAnalog turn{
      profile->getAnalogControlMapping(EControl::TANK_DRIVE_ARCADE_TURN)};
  switch (static_cast<ETankDriveTrainControlMode>(
      profile->getControlMode(EControlType::TANK_DRIVE_TRAIN))) {
    case ETankDriveTrainControlMode::TANK:
      updateTank();
      break;
    case ETankDriveTrainControlMode::ARCADE:
      updateArcade(linear, turn);
      break;
  }
}
}  // namespace tank_drive_train
}  // namespace op_control
}  // namespace driftless