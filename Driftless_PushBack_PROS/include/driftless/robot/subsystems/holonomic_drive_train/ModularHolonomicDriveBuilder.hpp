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
      m_modules;

 public:
  /// @brief Adds a module to the drive train
  /// @param module
  /// __std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>&__ The
  /// module to add
  /// @return __ModularHolonomicDriveBuilder*__ A pointer to this builder
  ModularHolonomicDriveBuilder* withModule(
      std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>& module);

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