#ifndef __I_HOLONOMIC_DRIVE_MODULE_HPP__
#define __I_HOLONOMIC_DRIVE_MODULE_HPP__

#include "driftless/robot/subsystems/holonomic_drive_train/HolonomicMotionVector.hpp"

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

/// @brief Interface for holonomic drive modules
/// @author Matthew Backman
class IHolonomicDriveModule {
 public:
  /// @brief Destroys the holonomic drive module object
  virtual ~IHolonomicDriveModule() = default;

  /// @brief Initializes the holonomic drive module
  virtual void init() = 0;

  /// @brief Runs the holonomic drive module
  virtual void run() = 0;

  /// @brief Sets the motion vector of the holonomic drive module
  /// @param motionVector __HolonomicMotionVector__ Vector representing the
  /// desired motion of the robot
  virtual void setMotionVector(HolonomicMotionVector motionVector) = 0;
};
}  // namespace holonomic_drive_module
}  // namespace holonomic_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif