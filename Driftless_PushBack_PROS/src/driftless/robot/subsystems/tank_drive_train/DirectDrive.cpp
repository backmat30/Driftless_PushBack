#include "driftless/robot/subsystems/tank_drive_train/DirectDrive.hpp"

#include "pros/screen.hpp"
namespace driftless {
namespace robot {
namespace subsystems {
namespace tank_drive_train {
void DirectDrive::init() {
  m_left_motors.init();
  m_right_motors.init();
}

void DirectDrive::run() {}

void DirectDrive::setVelocity(Velocity velocity) {
  m_left_motors.setVoltage(velocity.left_velocity * m_velocity_to_voltage);
  m_right_motors.setVoltage(velocity.right_velocity * m_velocity_to_voltage);
}

void DirectDrive::setVoltage(double left_voltage, double right_voltage) {
  m_left_motors.setVoltage(left_voltage);
  m_right_motors.setVoltage(right_voltage);
}

void DirectDrive::setLeftMotors(hal::MotorGroup& left_motors) {
  m_left_motors = left_motors;
}

void DirectDrive::setRightMotors(hal::MotorGroup& right_motors) {
  m_right_motors = right_motors;
}

void DirectDrive::setVelocityToVoltage(double velocity_to_voltage) {
  m_velocity_to_voltage = velocity_to_voltage;
}

void DirectDrive::setGearRatio(double gear_ratio) { m_gear_ratio = gear_ratio; }

void DirectDrive::setWheelRadius(double wheel_radius) {
  m_wheel_radius = wheel_radius;
}

void DirectDrive::setDriveRadius(double drive_radius) {
  m_drive_radius = drive_radius;
}

Velocity DirectDrive::getVelocity() {
  Velocity velocity{
      m_left_motors.getAngularVelocity() * m_wheel_radius / m_gear_ratio,
      m_right_motors.getAngularVelocity() * m_wheel_radius / m_gear_ratio};
  return velocity;
}

double DirectDrive::getDriveRadius() const { return m_drive_radius; }
}  // namespace tank_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless