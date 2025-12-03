#ifndef __I_TRAJECTORY_FOLLOWER_HPP__
#define __I_TRAJECTORY_FOLLOWER_HPP__

#include <memory>

#include "driftless/control/trajectory/TrajectoryPoint.hpp"
#include "driftless/robot/Robot.hpp"

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

/// @brief Interface for a generic trajectory follower
/// @author Matthew Backman
class ITrajectoryFollower {
 public:
  /// @brief Destroys the trajectory follower
  virtual ~ITrajectoryFollower() = default;

  /// @brief Initializes the trajectory follower
  virtual void init() = 0;

  /// @brief Runs the trajectory follower
  virtual void run() = 0;

  /// @brief Pauses the trajectory follower
  virtual void pause() = 0;

  /// @brief Resumes the trajectory follower
  virtual void resume() = 0;

  /// @brief Follows the given trajectory
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot to control
  /// @param trajectory __const std::vector<TrajectoryPoint>&__ The trajectory
  /// to follow
  virtual void followTrajectory(
      const std::shared_ptr<driftless::robot::Robot>& robot,
      const std::vector<TrajectoryPoint>& trajectory) = 0;

  /// @brief Checks if the target has been reached
  /// @return __bool__ True if the target has been reached, false otherwise
  virtual bool targetReached() = 0;
};
}  // namespace trajectory_follower
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif