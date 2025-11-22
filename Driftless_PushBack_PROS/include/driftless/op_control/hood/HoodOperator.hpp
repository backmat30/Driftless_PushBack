#ifndef __HOOD_OPERATOR_HPP__
#define __HOOD_OPERATOR_HPP__

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/hood/EHoodControlMode.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief The namespace for hood control
/// @author Matthew Backman
namespace hood {

/// @brief Class for controlling the hood during operator control
/// @author Matthew Backman
class HoodOperator {
 private:
  /// @brief The controller used by the driver
  std::shared_ptr<io::IController> m_controller{};

  /// @brief The robot being controlled
  std::shared_ptr<robot::Robot> m_robot{};

  /// @brief Sets the voltage of the hood motors
  /// @param voltage __double__ The voltage to set the motors to
  void setMotorVoltage(double voltage);

  /// @brief Toggles the height of the hood
  void toggleHoodHeight();

  /// @brief Toggles the gate of the hood
  void toggleHoodGate();

  /// @brief Updates the hood using toggles for pneumatic control
  /// @param spin_forwards __EControllerDigital__ The control for spinning the
  /// hood motors forwards from the front intake
  /// @param spin_forwards_back __EControllerDigital__ The control for spinning
  /// the hood motors forwards from the back intake
  /// @param spin_backwards __EControllerDigital__ The control for spinning the
  /// hood motors backwards
  /// @param toggle_height __EControllerDigital__ The control for toggling the
  /// hood height
  /// @param toggle_gate __EControllerDigital__ The control for toggling the
  /// hood gate
  void updateHoodToggle(EControllerDigital toggle_height,
                        EControllerDigital toggle_gate,
                        EControllerDigital toggle_descore,
                        EControllerDigital toggle_bump);

  void updateHoodSmartToggle(EControllerDigital toggle_high_goal,
                             EControllerDigital toggle_low_goal,
                             EControllerDigital toggle_gate,
                             EControllerDigital toggle_descore);

  void updateHoodRollers(EControllerDigital spin_forwards_front,
                         EControllerDigital spin_forwards_back,
                         EControllerDigital spin_backwards);

 public:
  /// @brief Constructs a new HoodOperator object
  /// @param controller __std::shared_ptr<io::IController>&__ The controller to
  /// use
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot to control
  HoodOperator(const std::shared_ptr<io::IController>& controller,
               const std::shared_ptr<robot::Robot>& robot);

  /// @brief Sets the hood operation based on the provided profile
  void update(const std::unique_ptr<profiles::IProfile>& profile);
};
}  // namespace hood
}  // namespace op_control
}  // namespace driftless
#endif