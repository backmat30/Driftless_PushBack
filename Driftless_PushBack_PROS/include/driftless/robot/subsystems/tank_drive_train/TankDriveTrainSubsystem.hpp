#ifndef __DRIVETRAIN_SUBSYSTEM_HPP__
#define __DRIVETRAIN_SUBSYSTEM_HPP__

#include <functional>
#include <map>
#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/tank_drive_train/ITankDriveTrain.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the drivetrain subsystem code
/// @author Matthew Backman
namespace tank_drive_train {

/// @brief The drive train subsystem
/// @author Matthew Backman
class TankDriveTrainSubsystem : public ASubsystem {
 private:
  std::unique_ptr<ITankDriveTrain> m_drive_train{};

 public:
  /// @brief Constructs a new drive train subsystem
  /// @param drivetrain __std::unique_ptr<IDrivetrain>&__ The drive train
  /// controller used by the subsystem
  TankDriveTrainSubsystem(std::unique_ptr<ITankDriveTrain> &drivetrain);

  /// @brief Initializes the subsystem
  void init() override;

  /// @brief Runs the subsystem
  void run() override;

  /// @brief Sends a command to the subsystem
  /// @param command_name __ESubsystemCommand__ The desired command
  /// @param args __va_list&__ The arguments for the command, if any
  void command(ESubsystemCommand command_name, va_list &args) override;

  /// @brief Gets a state of the subsystem
  /// @param state_name __ESubsystemState__ The desired state
  /// @return __void*__ A pointer to the state of the subsystem
  void *state(ESubsystemState state_name) override;
};
}  // namespace tank_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif