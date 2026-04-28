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

  /// @brief Sets the motion vector of the holonomic drive train with normalized
  /// vector
  /// @param motion_vector __HolonomicMotionVector__ the desired motion vector
  /// @details Magnitude must be within [0, 1]
  /// @details Angular velocity must be within [-1, 1]
  virtual void setNormalizedMotionVector(
      HolonomicMotionVector motion_vector) = 0;

  /// @brief Sets the motion vector of the holonimic drive train with normalized
  /// vector achieved through voltage control
  /// @param motion_vector __HolonomicMotionVector__ the desired motion vector
  virtual void setNormalizedMotionVectorVoltage(
      HolonomicMotionVector motion_vector) = 0;

  /// @brief Sets the voltage of the motors within the module
  /// @param voltage __double__ The voltage to set the motors to
  virtual void setRawVoltage(double voltage) = 0;

  /// @brief Gets the speed of the module in rad/s
  /// @return __double__ The speed of the module in rad/s
  virtual double getSpeed() = 0;
};
}  // namespace holonomic_drive_module
}  // namespace holonomic_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif