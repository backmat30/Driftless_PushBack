#ifndef __HOLONOMIC_DRIVE_TRAIN_SUBSYSTEM_HPP__
#define __HOLONOMIC_DRIVE_TRAIN_SUBSYSTEM_HPP__

#include <functional>
#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/IHolonomicDrive.hpp"
namespace driftless {
namespace robot {
namespace subsystems {
namespace holonomic_drive_train {

class HolonomicDriveTrainSubsystem : public ASubsystem {
 private:
  std::unique_ptr<IHolonomicDrive> m_drive_train{};

 public:
  HolonomicDriveTrainSubsystem(std::unique_ptr<IHolonomicDrive>& drive_train);
  void init() override;
  void run() override;
  void command(ESubsystemCommand command_name, va_list& args) override;
  void* state(ESubsystemState state_name) override;
};

}  // namespace holonomic_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif