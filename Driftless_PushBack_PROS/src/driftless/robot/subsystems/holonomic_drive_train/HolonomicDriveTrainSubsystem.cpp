#include "driftless/robot/subsystems/holonomic_drive_train/HolonomicDriveTrainSubsystem.hpp"

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
      double magnitude = va_arg(args, double);
      double direction = va_arg(args, double);
      double angular_velocity = va_arg(args, double);

      HolonomicMotionVector motion_vector{magnitude, direction,
                                          angular_velocity};
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
  }
}
void* HolonomicDriveTrainSubsystem::state(ESubsystemState state_name) {
  return nullptr;
}
}  // namespace driftless::robot::subsystems::holonomic_drive_train