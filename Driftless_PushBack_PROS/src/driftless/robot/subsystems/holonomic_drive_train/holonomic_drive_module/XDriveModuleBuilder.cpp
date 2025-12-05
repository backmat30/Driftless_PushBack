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

std::unique_ptr<IHolonomicDriveModule> XDriveModuleBuilder::build() {
  std::unique_ptr<XDriveModule> module = std::make_unique<XDriveModule>();
  module->setMotors(m_motors);
  module->setAngleOffset(m_angle_offset);
  return module;
}
}  // namespace
   // driftless::robot::subsystems::holonomic_drive_train::holonomic_drive_module