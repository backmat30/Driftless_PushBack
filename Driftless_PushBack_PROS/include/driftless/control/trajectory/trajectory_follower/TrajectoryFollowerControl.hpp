#ifndef __TRAJECTORY_FOLLOWER_CONTROL_HPP__
#define __TRAJECTORY_FOLLOWER_CONTROL_HPP__

#include "driftless/control/AControl.hpp"
#include "driftless/control/trajectory/trajectory_follower/ITrajectoryFollower.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for trajectory generation and following
/// @author Matthew Backman
namespace trajectory {

/// @brief Namespace for trajectory following algorithms
/// @author Matthew Backman
namespace trajectory_follower {

/// @brief Wrapper class for trajectory followers to adapt to control system
class TrajectoryFollowerControl : public AControl {
 private:
  std::unique_ptr<ITrajectoryFollower> m_trajectory_follower{};

 public:
  /// @brief Constructs a new TrajectoryFollowerControl
  /// @param trajectory_follower __std::unique_ptr<ITrajectoryFollower>&__ The trajectory follower to wrap
  TrajectoryFollowerControl(
      std::unique_ptr<ITrajectoryFollower>& trajectory_follower);

  /// @brief Initializes the TrajectoryFollowerControl
  void init() override;

  /// @brief Runs the TrajectoryFollowerControl
  void run() override;

  /// @brief Pauses the TrajectoryFollowerControl
  void pause() override;

  /// @brief Resumes the TrajectoryFollowerControl
  void resume() override;

  /// @brief Sends a command to the trajectory follower
  /// @param command_name __EControlCommand__ The command to follow
  /// @param args __va_list&__ The arguments of the command
  void command(EControlCommand command_name, va_list& args) override;

  /// @brief Gets a state of the trajectory follower
  /// @param state_name __EControlState__ The state to gather
  /// @return __void*__ The state of the trajectory follower
  void* state(EControlState state_name) override;
};
}  // namespace trajectory_follower
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif