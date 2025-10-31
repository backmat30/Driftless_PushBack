#include "driftless/op_control/holonomic_drive_train/HolonomicDriveTrainOperator.hpp"

namespace driftless::op_control::holonomic_drive_train {
void HolonomicDriveTrainOperator::updateDriveMotionVector(
    EControllerAnalog forward, EControllerAnalog strafe,
    EControllerAnalog turn) {
  double forward_input = m_controller->getAnalog(forward) / 127.0;
  double strafe_input = m_controller->getAnalog(strafe) / 127.0;
  double turn_input = m_controller->getAnalog(turn) / 127.0;

  // Create a motion vector based on controller inputs
  robot::subsystems::holonomic_drive_train::HolonomicMotionVector motion_vector;

  // Calculate magnitude and direction from forward and strafe inputs
  double magnitude =
      std::sqrt(forward_input * forward_input + strafe_input * strafe_input);

  if (magnitude > 1.0) {
    forward_input /= magnitude;
    strafe_input /= magnitude;
  }

  motion_vector.x = strafe_input;
  motion_vector.y = forward_input;

  // Set angular velocity from turn input
  motion_vector.angular_velocity = turn_input;

  // Send the motion vector to the holonomic drive train subsystem
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
                       robot::subsystems::ESubsystemCommand::
                           HOLONOMIC_DRIVE_TRAIN_SET_NORMALIZED_MOTION_VECTOR,
                       motion_vector);
}

HolonomicDriveTrainOperator::HolonomicDriveTrainOperator(
    const std::shared_ptr<io::IController>& controller,
    const std::shared_ptr<robot::Robot>& robot)
    : m_controller(controller), m_robot(robot) {}

void HolonomicDriveTrainOperator::setDriveMotionVector(
    const std::unique_ptr<profiles::IProfile>& profile) {
  EControllerAnalog forward_control =
      profile->getAnalogControlMapping(op_control::EControl::HOLONOMIC_FORWARD);
  EControllerAnalog strafe_control =
      profile->getAnalogControlMapping(op_control::EControl::HOLONOMIC_STRAFE);
  EControllerAnalog turn_control =
      profile->getAnalogControlMapping(op_control::EControl::HOLONOMIC_TURN);

  // Update the drive motion vector based on the profile
  updateDriveMotionVector(forward_control, strafe_control, turn_control);
}

}  // namespace driftless::op_control::holonomic_drive_train