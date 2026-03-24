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
  double linear_motor_velocity =
      linear_velocity * std::sqrt(2) * (3600 / m_motors.getGearRatio());

  // calculate the velocity contribution from angular velocity
  double turn_motor_velocity =
      motion_vector.angular_velocity * -(3600 / m_motors.getGearRatio());

  // Set the motor speeds (assuming a simple proportional control for
  // demonstration)
  double module_velocity = linear_motor_velocity + turn_motor_velocity;

  m_motors.setVelocity(module_velocity);
}

void XDriveModule::setRawVoltage(double voltage) {
  m_motors.setVoltage(voltage);
}

double XDriveModule::getSpeed() {
  double motor_speed = m_motors.getAngularVelocity();
  double gear_ratio = m_motors.getGearRatio();
  return motor_speed * gear_ratio;
}

void XDriveModule::setMotors(hal::MotorGroup& motors) { m_motors = motors; }

void XDriveModule::setAngleOffset(double angle_offset) {
  m_angle_offset = angle_offset;
}
}  // namespace
   // driftless::robot::subsystems::holonomic_drive_train::holonomic_drive_module