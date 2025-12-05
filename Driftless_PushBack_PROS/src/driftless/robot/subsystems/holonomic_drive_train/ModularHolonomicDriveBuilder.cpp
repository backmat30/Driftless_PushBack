#include "driftless/robot/subsystems/holonomic_drive_train/ModularHolonomicDriveBuilder.hpp"

namespace driftless::robot::subsystems::holonomic_drive_train {
ModularHolonomicDriveBuilder* ModularHolonomicDriveBuilder::withModule(
    std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>& module) {
  m_modules.push_back(std::move(module));
  return this;
}

ModularHolonomicDriveBuilder* ModularHolonomicDriveBuilder::withTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

ModularHolonomicDriveBuilder* ModularHolonomicDriveBuilder::withDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
  return this;
}

ModularHolonomicDriveBuilder* ModularHolonomicDriveBuilder::withMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return this;
}

ModularHolonomicDriveBuilder* ModularHolonomicDriveBuilder::
    withMaxLinearVelocity(double max_linear_velocity) {
  m_max_linear_velocity = max_linear_velocity;
  return this;
}

ModularHolonomicDriveBuilder* ModularHolonomicDriveBuilder::
    withMaxAngularVelocity(double max_angular_velocity) {
  m_max_angular_velocity = max_angular_velocity;
  return this;
}

std::unique_ptr<IHolonomicDrive> ModularHolonomicDriveBuilder::build() {
  std::unique_ptr<ModularHolonomicDrive> holonomic_drive = std::make_unique<ModularHolonomicDrive>();
  holonomic_drive->setModules(m_modules);
  holonomic_drive->setTask(m_task);
  holonomic_drive->setDelayer(m_delayer);
  holonomic_drive->setMutex(m_mutex);
  holonomic_drive->setMaxLinearVelocity(m_max_linear_velocity);
  holonomic_drive->setMaxAngularVelocity(m_max_angular_velocity);
  return holonomic_drive;
}
}