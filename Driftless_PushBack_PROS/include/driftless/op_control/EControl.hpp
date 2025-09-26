#ifndef __OP_CONTROL_E_CONTROL_HPP__
#define __OP_CONTROL_E_CONTROL_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Enumerated class for operator control commands
/// @author Matthew Backman
enum EControl {
  HOLONOMIC_FORWARD,
  HOLONOMIC_STRAFE,
  HOLONOMIC_TURN,
  TANK_DRIVE_ARCADE_LINEAR,
  TANK_DRIVE_ARCADE_TURN,
  INTAKE_FRONT_RUN_IN,
  INTAKE_FRONT_RUN_OUT,
  INTAKE_BACK_RUN_IN,
  INTAKE_BACK_TOGGLE_ARMS
};
}  // namespace op_control
}  // namespace driftless
#endif