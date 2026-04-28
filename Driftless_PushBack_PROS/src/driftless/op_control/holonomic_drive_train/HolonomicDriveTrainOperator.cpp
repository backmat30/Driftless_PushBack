#include "driftless/op_control/holonomic_drive_train/HolonomicDriveTrainOperator.hpp"

namespace driftless::op_control::holonomic_drive_train {
void HolonomicDriveTrainOperator::updateDriveMotionVector(
    EControllerAnalog forward, EControllerAnalog strafe,
    EControllerAnalog turn) {
  double forward_input = m_controller->getAnalog(forward) / 127.0;
  double strafe_input = m_controller->getAnalog(strafe) / 127.0;
  double turn_input = m_controller->getAnalog(turn) / 127.0;

  if (std::abs(forward_input) < 5 / 127.0) {
    forward_input = 0;
  }
  if (std::abs(strafe_input) < 5 / 127.0) {
    strafe_input = 0;
  }
  if (std::abs(turn_input) < 5 / 127.0) {
    turn_input = 0;
  }

  auto position =
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION));
  double heading = position.theta;

  if (!use_field_centric) {
    heading = M_PI / 2.0;
  }

  // Create a motion vector based on controller inputs
  robot::subsystems::holonomic_drive_train::HolonomicMotionVector motion_vector;

  // Calculate magnitude and direction from forward and strafe inputs
  double magnitude =
      std::sqrt(forward_input * forward_input + strafe_input * strafe_input);

  if (magnitude > 1.0) {
    forward_input /= magnitude;
    strafe_input /= magnitude;
  }

  motion_vector.x =
      strafe_input * std::sin(heading) - forward_input * std::cos(heading);
  motion_vector.y =
      forward_input * std::sin(heading) + strafe_input * std::cos(heading);

  // Set angular velocity from turn input
  motion_vector.angular_velocity = -turn_input;

  // Send the motion vector to the holonomic drive train subsystem
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
                       robot::subsystems::ESubsystemCommand::
                           HOLONOMIC_DRIVE_TRAIN_SET_NORMALIZED_LINEAR_VELOCITY,
                       motion_vector.x, motion_vector.y);

  if (std::abs(turn_input) > 0.1) {
    lock_direction = LockDirection::NONE;
    m_control_system->pause();
  }

  if (lock_direction == LockDirection::NONE) {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
        robot::subsystems::ESubsystemCommand::
            HOLONOMIC_DRIVE_TRAIN_SET_NORMALIZED_ANGULAR_VELOCITY,
        motion_vector.angular_velocity);
  }
}

void HolonomicDriveTrainOperator::updateHeadingLock(
    EControllerDigital lock_90, EControllerDigital lock_45) {
  bool start_lock_90{m_controller->getNewDigital(lock_90)};
  bool start_lock_45{m_controller->getNewDigital(lock_45)};
  bool turn_target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION, control::EControlState::TURN_TARGET_REACHED))};

  robot::subsystems::odometry::Position current_pos{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};
  double current_angle = current_pos.theta;

  double angle_difference{};

  if (start_lock_90) {
    double hemisphere_angle = current_angle;

    while (hemisphere_angle >= M_PI / 2.0) {
      hemisphere_angle -= M_PI;
    }
    while (hemisphere_angle < -M_PI / 2.0) {
      hemisphere_angle += M_PI;
    }

    lock_direction = LockDirection::NEAREST_180;
    turn_target_reached = false;
    angle_difference = (hemisphere_angle > 0.0)
                           ? M_PI / 2 - hemisphere_angle
                           : -M_PI / 2.0 - hemisphere_angle;
    target_angle = current_angle + angle_difference;
    m_control_system->sendCommand(control::EControl::MOTION,
                                  control::EControlCommand::TURN_TO_ANGLE,
                                  m_robot, M_PI * 3.0, target_angle,
                                  control::motion::ETurnDirection::AUTO);
  } else if (start_lock_45) {
    double quadrant_angle = current_angle;

    while (quadrant_angle >= M_PI / 2.0) {
      quadrant_angle -= M_PI / 2.0;
    }
    while (quadrant_angle < 0) {
      quadrant_angle += M_PI / 2.0;
    }

    lock_direction = LockDirection::NEAREST_45;
    turn_target_reached = false;
    angle_difference = M_PI / 4 - quadrant_angle;
    target_angle = current_angle + angle_difference;
    m_control_system->sendCommand(control::EControl::MOTION,
                                  control::EControlCommand::TURN_TO_ANGLE,
                                  m_robot, M_PI * 3.0, target_angle,
                                  control::motion::ETurnDirection::AUTO);
  }

  if (lock_direction != LockDirection::NONE && turn_target_reached) {
    if (std::abs(target_angle - current_angle) > M_PI / 45.0) {
      m_control_system->sendCommand(control::EControl::MOTION,
                                    control::EControlCommand::TURN_TO_ANGLE,
                                    m_robot, M_PI * 3.0, target_angle,
                                    control::motion::ETurnDirection::AUTO);
    }
  }
}

HolonomicDriveTrainOperator::HolonomicDriveTrainOperator(
    const std::shared_ptr<io::IController>& controller,
    const std::shared_ptr<robot::Robot>& robot,
    const std::shared_ptr<control::ControlSystem>& control_system)
    : m_controller(controller),
      m_robot(robot),
      m_control_system(control_system) {}

void HolonomicDriveTrainOperator::update(
    const std::unique_ptr<profiles::IProfile>& profile) {
  EControllerAnalog forward_control =
      profile->getAnalogControlMapping(op_control::EControl::HOLONOMIC_FORWARD);
  EControllerAnalog strafe_control =
      profile->getAnalogControlMapping(op_control::EControl::HOLONOMIC_STRAFE);
  EControllerAnalog turn_control =
      profile->getAnalogControlMapping(op_control::EControl::HOLONOMIC_TURN);
  EControllerDigital lock_90_control{profile->getDigitalControlMapping(
      op_control::EControl::HOLONOMIC_LOCK_90)};
  EControllerDigital lock_45_control{profile->getDigitalControlMapping(
      op_control::EControl::HOLONOMIC_LOCK_45)};
  EControllerDigital remove_field_centric{profile->getDigitalControlMapping(
      op_control::EControl::HOLONOMIC_CANCEL_FIELD_CENTRIC)};

  if (m_controller->getNewDigital(remove_field_centric)) {
    use_field_centric = false;
  }

  // Update the drive motion vector based on the profile
  updateHeadingLock(lock_90_control, lock_45_control);
  updateDriveMotionVector(forward_control, strafe_control, turn_control);
}

}  // namespace driftless::op_control::holonomic_drive_train