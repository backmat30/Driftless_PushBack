#include "driftless/robot/subsystems/holonomic_drive_train/HolonomicDriveTrainSubsystem.hpp"

#include "pros/screen.hpp"
namespace driftless::robot::subsystems::holonomic_drive_train {
HolonomicDriveTrainSubsystem::HolonomicDriveTrainSubsystem(
    std::unique_ptr<IHolonomicDrive>& drive_train)
    : m_drive_train(std::move(drive_train)),
      ASubsystem(ESubsystem::HOLONOMIC_DRIVE_TRAIN) {}

void HolonomicDriveTrainSubsystem::init() { m_drive_train->init(); }

void HolonomicDriveTrainSubsystem::run() { m_drive_train->run(); }

void HolonomicDriveTrainSubsystem::command(ESubsystemCommand command_name,
                                           va_list& args) {
  switch (command_name) {
    case ESubsystemCommand::HOLONOMIC_DRIVE_TRAIN_SET_MOTION_VECTOR: {
      double x = va_arg(args, double);
      double y = va_arg(args, double);
      double angular_velocity = va_arg(args, double);

      HolonomicMotionVector motion_vector{x, y, angular_velocity};
      m_drive_train->setMotionVector(motion_vector);
      break;
    }
    case ESubsystemCommand::
        HOLONOMIC_DRIVE_TRAIN_SET_NORMALIZED_MOTION_VECTOR: {
      double magnitude = va_arg(args, double);
      double direction = va_arg(args, double);
      double angular_velocity = va_arg(args, double);

      HolonomicMotionVector motion_vector{magnitude, direction,
                                          angular_velocity};
      m_drive_train->setNormalizedMotionVector(motion_vector);
      break;
    }
    case ESubsystemCommand::HOLONOMIC_DRIVE_TRAIN_SET_LINEAR_VELOCITY: {
      double x_velocity = va_arg(args, double);
      double y_velocity = va_arg(args, double);
      m_drive_train->setLinearVelocity(x_velocity, y_velocity);
      break;
    }
    case ESubsystemCommand::HOLONOMIC_DRIVE_TRAIN_SET_ANGULAR_VELOCITY: {
      double angular_velocity = va_arg(args, double);
      m_drive_train->setAngularVelocity(angular_velocity);
      break;
    }
    case ESubsystemCommand::
        HOLONOMIC_DRIVE_TRAIN_SET_NORMALIZED_LINEAR_VELOCITY: {
      double normalized_x_velocity = va_arg(args, double);
      double normalized_y_velocity = va_arg(args, double);
      m_drive_train->setNormalizedLinearVelocity(normalized_x_velocity,
                                                 normalized_y_velocity);
      break;
    }
    case ESubsystemCommand::
        HOLONOMIC_DRIVE_TRAIN_SET_NORMALIZED_ANGULAR_VELOCITY: {
      double normalized_angular_velocity = va_arg(args, double);
      m_drive_train->setNormalizedAngularVelocity(normalized_angular_velocity);
      break;
    }
    case ESubsystemCommand::HOLONOMIC_DRIVE_TRAIN_SET_MODULE_VOLTAGE: {
      int wheel = va_arg(args, int);
      double voltage = va_arg(args, double);
      m_drive_train->setWheelVoltage(wheel, voltage);
      break;
    }
    case ESubsystemCommand::HOLONOMIC_DRIVE_TRAIN_SET_CONTROL_TYPE: {
      IHolonomicDrive::ControlType type{
          va_arg(args, IHolonomicDrive::ControlType)};
      m_drive_train->setControlType(type);
    }
  }
}
void* HolonomicDriveTrainSubsystem::state(ESubsystemState state_name) {
  void* out = nullptr;
  switch (state_name) {
    case ESubsystemState::HOLONOMIC_DRIVE_TRAIN_GET_WHEEL_SPEED: {
      double* wheel_speeds = new double[4];
      for (int i = 0; i < 4; ++i) {
        wheel_speeds[i] = m_drive_train->getWheelSpeed(i);
      }
      out = wheel_speeds;
      break;
    }
  }
  return out;
}
}  // namespace driftless::robot::subsystems::holonomic_drive_train