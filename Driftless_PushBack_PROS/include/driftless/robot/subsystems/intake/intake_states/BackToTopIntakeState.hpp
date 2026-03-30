#ifndef __BACK_TO_TOP_INTAKE_STATE_HPP__
#define __BACK_TO_TOP_INTAKE_STATE_HPP__

#include "driftless/robot/subsystems/intake/StateMachineIntake.hpp"
#include "driftless/robot/subsystems/intake/intake_states/IIntakeState.hpp"

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

class BackToTopIntakeState : public IIntakeState {
 public:
  /// @brief Updates the intake to run the back intake motors towards the hood
  /// @param intake __StateMachineIntake&__ The intake being updated
  void update(StateMachineIntake& intake) override;
};

}  // namespace intake_states
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif