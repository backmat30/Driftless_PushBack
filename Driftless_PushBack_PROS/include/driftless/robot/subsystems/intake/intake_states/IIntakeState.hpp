#ifndef __I_INTAKE_STATE_HPP__
#define __I_INTAKE_STATE_HPP__

#include "driftless/robot/subsystems/intake/StateMachineIntake.hpp"
#include "driftless/robot/subsystems/intake/intake_states/EIntakeStates.hpp"

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

class IIntakeState {
 public:
  virtual ~IIntakeState() = default;

  virtual void update(StateMachineIntake& intake) = 0;

  virtual EIntakeStates processStateChange(StateMachineIntake& intake) = 0;
};

}  // namespace intake_states
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif