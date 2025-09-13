#include "driftless/robot/subsystems/holonomic_drive_train/ModularHolonomicDriveBuilder.hpp"

namespace driftless::robot::subsystems::holonomic_drive_train {
ModularHolonomicDriveBuilder* ModularHolonomicDriveBuilder::withModule(
    std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>& module) {
  m_modules.push_back(std::move(module));
  return this;
}

std::unique_ptr<IHolonomicDrive> ModularHolonomicDriveBuilder::build() {
  std::unique_ptr<ModularHolonomicDrive> holonomic_drive = std::make_unique<ModularHolonomicDrive>();
  holonomic_drive->setModules(m_modules);
  return holonomic_drive;
}
}