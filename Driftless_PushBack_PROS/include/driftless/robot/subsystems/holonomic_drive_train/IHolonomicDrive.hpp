#ifndef __I_HOLONOMIC_DRIVE_HPP__
#define __I_HOLONOMIC_DRIVE_HPP__

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

/// @brief Interface for holonomic drive train systems
/// @author Matthew Backman
class IHolonomicDrive {
 public:
  enum class ControlType { VELOCITY, VOLTAGE };

  /// @brief Destroys the holonomic drive object
  virtual ~IHolonomicDrive() = default;

  /// @brief Initializes the holonomic drive object
  virtual void init() = 0;

  /// @brief Runs the holonomic drive object
  virtual void run() = 0;

  /// @brief Sets the motion vector of the holonomic drive train
  /// @param motion_vector __HolonomicMotionVector__ the desired motion vector
  virtual void setMotionVector(HolonomicMotionVector motion_vector) = 0;

  /// @brief Sets the X velocity of the holonomic drive train
  /// @param x_velocity The desired X velocity
  /// @param y_velocity The desired Y velocity
  virtual void setLinearVelocity(double x_velocity, double y_velocity) = 0;

  /// @brief Sets the angular velocity of the holonomic drive train
  /// @param angular_velocity The desired angular velocity
  virtual void setAngularVelocity(double angular_velocity) = 0;

  /// @brief Sets the motion vector of the holonomic drive train with normalized
  /// vector
  /// @param motion_vector __HolonomicMotionVector__ the desired motion vector
  /// @details Magnitude must be within [0, 1]
  /// @details Angular velocity must be within [-1, 1]
  virtual void setNormalizedMotionVector(
      HolonomicMotionVector motion_vector) = 0;

  /// @brief Sets the normalized X velocity of the holonomic drive train
  /// @param x_velocity The desired normalized X velocity [-1, 1]
  /// @param y_velocity The desired normalized Y velocity [-1, 1]
  virtual void setNormalizedLinearVelocity(double x_velocity,
                                           double y_velocity) = 0;

  /// @brief Sets the normalized angular velocity of the holonomic drive train
  /// @param angular_velocity The desired normalized angular velocity [-1, 1]
  virtual void setNormalizedAngularVelocity(double angular_velocity) = 0;

  /// @brief Sets the voltage of a specific wheel in the holonomic drive train
  /// Only use for tuning or testing, not for regular control of the drive train
  /// @param wheel __int__ The index of the wheel to set the voltage of
  /// @param voltage __double__ The voltage to set the wheel to
  virtual void setWheelVoltage(int wheel, double voltage) = 0;

  virtual void setControlType(ControlType control_type) = 0;

  /// @brief Gets the speed of a specific wheel in the holonomic drive train
  /// @param wheel __int__ The index of the wheel to get the speed of
  /// @return __double__ The speed of the specified wheel in rad/s
  virtual double getWheelSpeed(int wheel) = 0;
};
}  // namespace holonomic_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif