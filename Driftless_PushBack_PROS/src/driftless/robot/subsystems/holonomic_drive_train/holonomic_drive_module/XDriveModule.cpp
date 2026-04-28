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
      linear_velocity * std::sqrt(2) * (120 * M_PI / m_motors.getGearRatio());

  // calculate the velocity contribution from angular velocity
  double turn_motor_velocity =
      motion_vector.angular_velocity * -(120 * M_PI / m_motors.getGearRatio());

  // Set the motor speeds (assuming a simple proportional control for
  // demonstration)
  double module_velocity = linear_motor_velocity + turn_motor_velocity;

  m_motors.setVelocity(module_velocity);
}

void XDriveModule::setNormalizedMotionVectorVoltage(
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
  double linear_motor_voltage = linear_velocity * std::sqrt(2) * 10.25;
  if (linear_motor_voltage != 0) {
    linear_motor_voltage +=
        1.75 * (linear_velocity / std::abs(linear_velocity));
  }

  // calculate the velocity contribution from angular velocity
  double turn_motor_voltage = motion_vector.angular_velocity * -11.0;
  if (turn_motor_voltage != 0) {
    turn_motor_voltage += 1.0 * (motion_vector.angular_velocity /
                                 std::abs(motion_vector.angular_velocity));
  }

  // Set the motor speeds (assuming a simple proportional control for
  // demonstration)
  double module_voltage = linear_motor_voltage + turn_motor_voltage;

  m_motors.setVoltage(module_voltage);
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