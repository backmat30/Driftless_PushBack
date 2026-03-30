#ifndef __E_INTAKE_STATES_HPP__
#define __E_INTAKE_STATES_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for the intake subsystem
/// @author Matthew Backman
namespace intake {

/// @brief Namespace for the various states the intake can be in while running
/// @author Matthew Backman
namespace intake_states {

/// @brief Enum containing the various states the intake can be in
/// @author Matthew Backman
enum class EIntakeStates {
  IDLE = 0,
  FRONT_IN = 1,
  FRONT_OUT = 2,
  BACK_IN_BOTTOM = 3,
  BACK_IN_TOP = 4,
};
}  // namespace intake_states
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif