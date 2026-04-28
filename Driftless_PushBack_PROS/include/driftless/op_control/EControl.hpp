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
  HOLONOMIC_LOCK_90,
  HOLONOMIC_LOCK_45,
  HOLONOMIC_CANCEL_FIELD_CENTRIC,
  TANK_DRIVE_ARCADE_LINEAR,
  TANK_DRIVE_ARCADE_TURN,
  INTAKE_FRONT_RUN_IN,
  INTAKE_FRONT_RUN_OUT,
  INTAKE_BACK_RUN_IN,
  INTAKE_BACK_TO_BOTTOM,
  INTAKE_BACK_TOGGLE_ARMS,
  HOOD_TOGGLE_RAISED,
  HOOD_TOGGLE_GATE,
  HOOD_TOGGLE_DESCORE,
  HOOD_TOGGLE_BUMP,
  BRAKE_TOGGLE,
  RAKE_TOGGLE
};
}  // namespace op_control
}  // namespace driftless
#endif