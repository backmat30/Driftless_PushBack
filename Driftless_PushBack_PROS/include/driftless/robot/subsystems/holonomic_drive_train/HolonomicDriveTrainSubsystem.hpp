#ifndef __HOLONOMIC_DRIVE_TRAIN_SUBSYSTEM_HPP__
#define __HOLONOMIC_DRIVE_TRAIN_SUBSYSTEM_HPP__

#include <functional>
#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/IHolonomicDrive.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the holonomic drive train subsystem code
/// @author Matthew Backman
namespace holonomic_drive_train {

/// @brief The holonomic drive train subsystem
/// @author Matthew Backman
class HolonomicDriveTrainSubsystem : public ASubsystem {
 private:
  /// @brief The holonomic drive train object being wrapped
  std::unique_ptr<IHolonomicDrive> m_drive_train{};

 public:
  /// @brief Constructor
  /// @param drive_train __std::unique_ptr<IHolonomicDrive>&__ The holonomic drive
  /// train object to wrap
  HolonomicDriveTrainSubsystem(std::unique_ptr<IHolonomicDrive>& drive_train);

  /// @brief Initializes the holonomic drive train subsystem
  void init() override;

  /// @brief Runs the holonomic drive train subsystem
  void run() override;

  /// @brief Sends a command to the holonomic drive train subsystem
  /// @param command_name __ESubsystemCommand__ The command to send
  /// @param args __va_list&__ The arguments for the command
  void command(ESubsystemCommand command_name, va_list& args) override;

  /// @brief Gets the state of the holonomic drive train subsystem
  /// @param state_name __ESubsystemState__ The state to get
  /// @return __void*__ The state data
  void* state(ESubsystemState state_name) override;
};

}  // namespace holonomic_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif