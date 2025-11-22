#ifndef __E_HOOD_CONTROL_MODE_HPP__
#define __E_HOOD_CONTROL_MODE_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for operator control management
/// @author Matthew Backman
namespace op_control {

/// @brief The namespace for hood control
/// @author Matthew Backman
namespace hood {

  enum class EHoodControlMode {
    SPLIT_TOGGLE,
    SMART_TOGGLE
  };
}}}

#endif