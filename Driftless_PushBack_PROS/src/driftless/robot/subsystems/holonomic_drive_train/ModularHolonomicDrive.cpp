#include "driftless/robot/subsystems/holonomic_drive_train/ModularHolonomicDrive.hpp"

namespace driftless::robot::subsystems::holonomic_drive_train {
void ModularHolonomicDrive::init() {
  for (auto& module : m_modules) {
    if (module) {
      module->init();
    }
  }
}

void ModularHolonomicDrive::run() {
  for (auto& module : m_modules) {
    if (module) {
      module->run();
    }
  }
}

void ModularHolonomicDrive::setMotionVector(
    HolonomicMotionVector motionVector) {
  for (auto& module : m_modules) {
    if (module) {
      module->setMotionVector(motionVector);
    }
  }
}

void ModularHolonomicDrive::setNormalizedMotionVector(
    HolonomicMotionVector motion_vector) {
  for (auto& module : m_modules) {
    if (module) {
      module->setNormalizedMotionVector(motion_vector);
    }
  }
}

void ModularHolonomicDrive::setModules(
    std::vector<std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>>&
        modules) {
  m_modules = std::move(modules);
}
}  // namespace driftless::robot::subsystems::holonomic_drive_train