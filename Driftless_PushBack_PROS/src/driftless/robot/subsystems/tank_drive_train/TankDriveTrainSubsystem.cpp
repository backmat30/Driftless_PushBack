#include "driftless/robot/subsystems/tank_drive_train/TankDriveTrainSubsystem.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace tank_drive_train {
TankDriveTrainSubsystem::TankDriveTrainSubsystem(
    std::unique_ptr<ITankDriveTrain>& drivetrain)
    : ASubsystem{ESubsystem::DRIVETRAIN}, m_drive_train(std::move(drivetrain)) {}

void TankDriveTrainSubsystem::init() { m_drive_train->init(); }

void TankDriveTrainSubsystem::run() { m_drive_train->run(); }

void TankDriveTrainSubsystem::command(ESubsystemCommand command_name,
                                  va_list& args) {
  if (command_name == ESubsystemCommand::DRIVETRAIN_SET_VELOCITY) {
    double left_velocity{va_arg(args, double)};
    double right_velocity{va_arg(args, double)};
    Velocity velocity{left_velocity, right_velocity};
    m_drive_train->setVelocity(velocity);
  } else if (command_name == ESubsystemCommand::DRIVETRAIN_SET_VOLTAGE) {
    double left_voltage{va_arg(args, double)};
    double right_voltage{va_arg(args, double)};
    m_drive_train->setVoltage(left_voltage, right_voltage);
  }
}

void* TankDriveTrainSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};

  if (state_name == ESubsystemState::DRIVETRAIN_GET_VELOCITY) {
    Velocity* velocity{new Velocity{m_drive_train->getVelocity()}};
    result = velocity;
  } else if (state_name == ESubsystemState::DRIVETRAIN_GET_RADIUS) {
    double* radius{new double{m_drive_train->getDriveRadius()}};
    result = radius;
  }
  return result;
}
}  // namespace drivetrain
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless