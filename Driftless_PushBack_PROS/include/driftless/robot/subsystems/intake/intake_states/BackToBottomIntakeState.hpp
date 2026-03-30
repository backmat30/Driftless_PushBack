#ifndef __BACK_TO_BOTTOM_INTAKE_STATE_HPP__
#define __BACK_TO_BOTTOM_INTAKE_STATE_HPP__

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

/// @brief Intake state for running the front intake motors forward
/// @author Matthew Backman
class BackToBottomIntakeState : public IIntakeState {
 private:
  bool m_has_first_matchloader_block{};

  bool m_ready_for_second_matchloader_block{};

  bool m_has_second_matchloader_block{};

  double m_first_matchloader_block_pos{-__DBL_MAX__};

 public:
  /// @brief Updates the intake to run the front intake motors forward
  /// @param intake __StateMachineIntake&__ The intake being updated
  void update(StateMachineIntake& intake) override;
};

}  // namespace intake_states
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif