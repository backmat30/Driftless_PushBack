#ifndef __HOLONOMIC_DRIVE_TRAIN_OPERATOR_HPP__
#define __HOLONOMIC_DRIVE_TRAIN_OPERATOR_HPP__

#include <memory>
#include <cmath>

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerAnalog.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/HolonomicMotionVector.hpp"
#include "driftless/robot/subsystems/odometry/Position.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief The namespace for holonomic drive train control
/// @author Matthew Backman
namespace holonomic_drive_train {

/// @brief Class for controlling a holonomic drive train during operator control
/// @author Matthew Backman
class HolonomicDriveTrainOperator {
 private:
  /// @brief The controller used by the driver
  std::shared_ptr<io::IController> m_controller{};

  /// @brief The robot being controlled
  std::shared_ptr<robot::Robot> m_robot{};

  /// @brief Updates the drive motion vector based on controller inputs
  /// @param forward __EControllerAnalog__ The analog input for forward/backward
  /// @param strafe __EControllerAnalog__ The analog input for left/right
  /// @param turn __EControllerAnalog__ The analog input for rotation
  void updateDriveMotionVector(EControllerAnalog forward,
                               EControllerAnalog strafe,
                               EControllerAnalog turn);

 public:
  /// @brief Constructs a new HolonomicDriveTrainOperator object
  /// @param controller __std::shared_ptr<io::IController>&__ The controller to use
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot to control
  HolonomicDriveTrainOperator(
      const std::shared_ptr<io::IController>& controller,
      const std::shared_ptr<robot::Robot>& robot);

  /// @brief Sets the drive motion vector based on the provided profile
  void setDriveMotionVector(const std::unique_ptr<profiles::IProfile>& profile);
};
}  // namespace holonomic_drive_train
}  // namespace op_control
}  // namespace driftless

#endif