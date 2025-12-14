#ifndef __E_INTAKE_CONTROL_MODE_HPP__
#define __E_INTAKE_CONTROL_MODE_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief Namespace for intake control during operator control
/// @author Matthew Backman
namespace intake {
enum class EIntakeControlMode {
  SPLIT,
  SMART_SPLIT
};
}
}  // namespace op_control
}  // namespace driftless

#endif