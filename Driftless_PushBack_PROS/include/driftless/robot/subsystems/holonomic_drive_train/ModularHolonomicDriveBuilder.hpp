#ifndef __MODULAR_HOLONOMIC_DRIVE_BUILDER_HPP__
#define __MODULAR_HOLONOMIC_DRIVE_BUILDER_HPP__

#include <memory>
#include <vector>

#include "driftless/robot/subsystems/holonomic_drive_train/ModularHolonomicDrive.hpp"

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

/// @brief Builder class for creating a ModularHolonomicDrive
/// @author Matthew Backman
class ModularHolonomicDriveBuilder {
 private:
  /// @brief The modules to be added to the drive train
  std::vector<std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>>
      m_modules{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  double m_max_linear_velocity{};

  double m_max_angular_velocity{};

 public:
  /// @brief Adds a module to the drive train
  /// @param module
  /// __std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>&__ The
  /// module to add
  /// @return __ModularHolonomicDriveBuilder*__ A pointer to this builder
  ModularHolonomicDriveBuilder* withModule(
      std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>& module);

  /// @brief Adds a task to the drive train
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task to be used
  /// @return __ModularHolonomicDriveBuilder*__ A pointer to this builder
  ModularHolonomicDriveBuilder* withTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Adds a delayer to the drive train
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer to be used
  /// @return __ModularHolonomicDriveBuilder*__ A pointer to this builder
  ModularHolonomicDriveBuilder* withDelayer(
      std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Adds a mutex to the drive train
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex to be used
  /// @return __ModularHolonomicDriveBuilder*__ A pointer to this builder
  ModularHolonomicDriveBuilder* withMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Adds a maximum linear velocity to the drive train
  /// @param max_linear_velocity __double__ The maximum linear velocity to be used
  /// @return __ModularHolonomicDriveBuilder*__ A pointer to this builder
  ModularHolonomicDriveBuilder* withMaxLinearVelocity(
      double max_linear_velocity);

  /// @brief Adds a maximum angular velocity to the drive train
  /// @param max_angular_velocity __double__ The maximum angular velocity to be used
  /// @return __ModularHolonomicDriveBuilder*__ A pointer to this builder
  ModularHolonomicDriveBuilder* withMaxAngularVelocity(
      double max_angular_velocity);

  /// @brief Builds the ModularHolonomicDrive
  /// @return __std::unique_ptr<IHolonomicDrive>__ A unique pointer to the
  /// created ModularHolonomicDrive
  std::unique_ptr<IHolonomicDrive> build();
};

}  // namespace holonomic_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif