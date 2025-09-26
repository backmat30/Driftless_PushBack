#ifndef __INTAKE_OPERATOR_HPP__
#define __INTAKE_OPERATOR_HPP__

#include <memory>

#include "driftless/io/IController.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for intake control during operator control
/// @author Matthew Backman
namespace intake {

/// @brief Class for controlling the intake during the operator control period
/// @author Matthew Backman
class IntakeOperator {
 private:
  /// @brief The controller used by the driver
  std::shared_ptr<io::IController> m_controller{};

  /// @brief The robot being controlled
  std::shared_ptr<robot::Robot> m_robot{};

  /// @brief Updates the intake using split controls
  /// @param front_intake_in __EControllerDigital__ The digital input tied to
  /// spinning the front intake forward
  /// @param front_intake_out __EControllerDigital__ The digital input tied to
  /// spinning the front intake backward
  /// @param back_intake_in __EControllerDigital__ The digital input tied to
  /// spinning the back intake forward
  /// @param back_arms_toggle __EControllerDigital__ The digital input tied to
  /// toggling the intake arms
  void updateIntakeSplit(EControllerDigital front_intake_in,
                         EControllerDigital front_intake_out,
                         EControllerDigital back_intake_in,
                         EControllerDigital back_arms_toggle);

 public:
  /// @brief Constructs a new IntakeOperator object
  /// @param controller __const std::shared_ptr<io::IController>&__ The
  /// controller to use
  /// @param robot __const std::shared_ptr<robot::Robot>&__ The robot to control
  IntakeOperator(const std::shared_ptr<io::IController>& controller,
                 const std::shared_ptr<robot::Robot>& robot);

  /// @brief Updates the intake using the given driver profile
  /// @param profile __const std::unique_ptr<profiles::IProfile>&__ The driver's
  /// controller profile
  void update(const std::unique_ptr<profiles::IProfile>& profile);
};
}  // namespace intake
}  // namespace op_control
}  // namespace driftless
#endif