#ifndef __X_DRIVE_MODULE_HPP__
#define __X_DRIVE_MODULE_HPP__

#include <cmath>

#include "driftless/hal/MotorGroup.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/holonomic_drive_module/IHolonomicDriveModule.hpp"

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

/// @brief Class representing an X-drive module
/// @author Matthew Backman
class XDriveModule : public IHolonomicDriveModule {
 private:
  /// @brief The motors in the module
  hal::MotorGroup m_motors{};

  /// @brief The angle offset of the module in radians
  double m_angle_offset{};

 public:
  /// @brief Initializes the X-drive module
  void init() override;

  /// @brief Runs the X-drive module
  void run() override;

  /// @brief Sets the motion vector of the holonomic drive train with normalized
  /// vector
  /// @param motion_vector __HolonomicMotionVector__ the desired motion vector
  /// @details Magnitude must be within [0, 1]
  /// @details Angular velocity must be within [-1, 1]
  void setNormalizedMotionVector(HolonomicMotionVector motion_vector) override;

  /// @brief Sets the motion vector of the holonimic drive train with normalized
  /// vector achieved through voltage control
  /// @param motion_vector __HolonomicMotionVector__ the desired motion vector
  void setNormalizedMotionVectorVoltage(
      HolonomicMotionVector motion_vector) override;

  /// @brief Sets the voltage of the motors within the module
  /// @param voltage __double__ The voltage to set the motors to
  void setRawVoltage(double voltage) override;

  /// @brief Sets the motors of the X-drive module
  /// @param motors __MotorGroup__ The motors to be used by the module
  void setMotors(hal::MotorGroup& motors);

  /// @brief Sets the angle offset of the X-drive module
  /// @param angleOffset __double__ The angle offset to be set
  void setAngleOffset(double angle_offset);

  /// @brief Gets the speed of the module in rad/s
  /// @return __double__ The speed of the module in rad/s
  double getSpeed() override;
};
}  // namespace holonomic_drive_module
}  // namespace holonomic_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif