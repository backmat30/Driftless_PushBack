#ifndef __X_DRIVE_MODULE_BUILDER_HPP__
#define __X_DRIVE_MODULE_BUILDER_HPP__

#include <memory>

#include "driftless/robot/subsystems/holonomic_drive_train/holonomic_drive_module/XDriveModule.hpp"

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

/// @brief The namespace for the holonomic drive module code
/// @author Matthew Backman
namespace holonomic_drive_module {

/// @brief Builder class for creating an XDriveModule
/// @author Matthew Backman
class XDriveModuleBuilder {
 private:
  /// @brief The motors used to create the module
  hal::MotorGroup m_motors{};

  /// @brief The offset angle of the module
  double m_angle_offset{};

 public:
  /// @brief Adds a motor to the module
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor to add
  /// @return __XDriveModuleBuilder*__ A pointer to this builder
  XDriveModuleBuilder* withMotor(std::unique_ptr<io::IMotor>& motor);

  /// @brief Sets the angle offset of the module
  /// @param angle_offset __double__ The angle offset in radians
  /// @return __XDriveModuleBuilder*__ A pointer to this builder
  XDriveModuleBuilder* withAngleOffset(double angle_offset);

  /// @brief Builds the XDriveModule
  /// @return __std::unique_ptr<IHolonomicDriveModule>__ A unique pointer to the
  /// created XDriveModule
  std::unique_ptr<IHolonomicDriveModule> build();
};
}  // namespace holonomic_drive_module
}  // namespace holonomic_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif