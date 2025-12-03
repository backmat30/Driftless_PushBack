#include "driftless/robot/subsystems/holonomic_drive_train/holonomic_drive_module/XDriveModule.hpp"

namespace driftless::robot::subsystems::holonomic_drive_train::
    holonomic_drive_module {
void XDriveModule::init() { m_motors.init(); }

void XDriveModule::run() {}

void XDriveModule::setNormalizedMotionVector(
    HolonomicMotionVector motion_vector) {
  // Clamp the x and y to [-1, 1]
  if (motion_vector.x > 1.0) {
    motion_vector.x = 1.0;
  } else if (motion_vector.x < -1.0) {
    motion_vector.x = -1.0;
  }

  if (motion_vector.y > 1.0) {
    motion_vector.y = 1.0;
  } else if (motion_vector.y < -1.0) {
    motion_vector.y = -1.0;
  }

  // Clamp the angular velocity to [-1, 1]
  if (motion_vector.angular_velocity > 1.0) {
    motion_vector.angular_velocity = 1.0;
  } else if (motion_vector.angular_velocity < -1.0) {
    motion_vector.angular_velocity = -1.0;
  }

  // Calculate the y-component of the vector, aka forward velocity of the wheel
  double linear_velocity = std::sin(m_angle_offset) * motion_vector.y +
                           std::cos(m_angle_offset) * motion_vector.x;
  double linear_voltage = linear_velocity * 12.0 * std::sqrt(2);

  // calculate the velocity contribution from angular velocity
  double turn_voltage = motion_vector.angular_velocity * 12.0;

  // Set the motor speeds (assuming a simple proportional control for
  // demonstration)
  double module_voltage = linear_voltage + turn_voltage;

  m_motors.setVoltage(module_voltage);
}

void XDriveModule::setMotors(hal::MotorGroup& motors) { m_motors = motors; }

void XDriveModule::setAngleOffset(double angle_offset) {
  m_angle_offset = angle_offset;
}

void XDriveModule::setMaxAngularVelocity(double max_angular_velocity) {
  m_max_angular_velocity = max_angular_velocity;
}

void XDriveModule::setMaxLinearVelocity(double max_linear_velocity) {
  m_max_linear_velocity = max_linear_velocity;
}
}  // namespace
   // driftless::robot::subsystems::holonomic_drive_train::holonomic_drive_module