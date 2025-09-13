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

void ModularHolonomicDrive::addModule(
    std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>& module) {
  m_modules.push_back(std::move(module));
}
}  // namespace driftless::robot::subsystems::holonomic_drive_train