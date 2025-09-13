#include "driftless/robot/subsystems/holonomic_drive_train/holonomic_drive_module/XDriveModule.hpp"

namespace driftless::robot::subsystems::holonomic_drive_train::
    holonomic_drive_module {
void XDriveModule::init() { m_motors.init(); }

void XDriveModule::run() {}

void XDriveModule::setMotionVector(HolonomicMotionVector motion_vector) {
  // Create a motion vector using the angular offset of the module
  HolonomicMotionVector module_motion_vector = motion_vector;
  module_motion_vector.direction -= m_angle_offset;

  // Calculate the y-component of the vector, aka forward velocity of the wheel
  double linear_velocity =
      module_motion_vector.magnitude * std::sin(module_motion_vector.direction);
  double linear_voltage = linear_velocity * m_linear_velocity_to_voltage;

  // calculate the velocity contribution from angular velocity
  double turn_voltage =
      module_motion_vector.angular_velocity * m_angular_velocity_to_voltage;

  // Set the motor speeds (assuming a simple proportional control for
  // demonstration)
  double module_velocity = linear_voltage + turn_voltage;
  m_motors.setVoltage(module_velocity);
}

void XDriveModule::setMotors(hal::MotorGroup& motors) { m_motors = motors; }

void XDriveModule::setAngleOffset(double angle_offset) {
  m_angle_offset = angle_offset;
}

void XDriveModule::setAngularVelocityToVoltage(
    double angular_velocity_to_voltage) {
  m_angular_velocity_to_voltage = angular_velocity_to_voltage;
}

/// @brief Sets the linear velocity to voltage conversion factor
/// @param linear_velocity_to_voltage __double__ The conversion factor to be set
void XDriveModule::setLinearVelocityToVoltage(
    double linear_velocity_to_voltage) {
  m_linear_velocity_to_voltage = linear_velocity_to_voltage;
}
}  // namespace
   // driftless::robot::subsystems::holonomic_drive_train::holonomic_drive_module