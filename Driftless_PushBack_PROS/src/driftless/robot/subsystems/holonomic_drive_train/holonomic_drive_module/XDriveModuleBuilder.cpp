#include "driftless/robot/subsystems/holonomic_drive_train/holonomic_drive_module/XDriveModuleBuilder.hpp"

namespace driftless::robot::subsystems::holonomic_drive_train::
    holonomic_drive_module {
XDriveModuleBuilder* XDriveModuleBuilder::withMotor(
    std::unique_ptr<io::IMotor>& motor) {
  m_motors.addMotor(motor);
  return this;
}

XDriveModuleBuilder* XDriveModuleBuilder::withAngleOffset(double angle_offset) {
  m_angle_offset = angle_offset;
  return this;
}

XDriveModuleBuilder* XDriveModuleBuilder::withAngularVelocityToVoltage(
    double angular_velocity_to_voltage) {
  m_angular_velocity_to_voltage = angular_velocity_to_voltage;
  return this;
}

XDriveModuleBuilder* XDriveModuleBuilder::withLinearVelocityToVoltage(
    double linear_velocity_to_voltage) {
  m_linear_velocity_to_voltage = linear_velocity_to_voltage;
  return this;
}

std::unique_ptr<IHolonomicDriveModule> XDriveModuleBuilder::build() {
  std::unique_ptr<XDriveModule> module = std::make_unique<XDriveModule>();
  module->setMotors(m_motors);
  module->setAngleOffset(m_angle_offset);
  module->setAngularVelocityToVoltage(m_angular_velocity_to_voltage);
  module->setLinearVelocityToVoltage(m_linear_velocity_to_voltage);
  return module;
}
}  // namespace
   // driftless::robot::subsystems::holonomic_drive_train::holonomic_drive_module