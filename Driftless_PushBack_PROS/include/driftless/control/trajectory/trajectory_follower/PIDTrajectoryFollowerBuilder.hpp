#ifndef __PID_TRAJECTORY_FOLLOWER_BUILDER_HPP__
#define __PID_TRAJECTORY_FOLLOWER_BUILDER_HPP__

#include "driftless/control/trajectory/trajectory_follower/PIDTrajectoryFollower.hpp"

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

/// @brief Builder class for PIDTrajectoryFollower objects
class PIDTrajectoryFollowerBuilder {
 private:
  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<rtos::IClock> m_clock{};

  PID m_x_pid{};

  PID m_y_pid{};

  PID m_theta_pid{};

  double m_target_tolerance{};

  double m_target_velocity{};

 public:
  /// @brief Sets the delayer used by the trajectory follower
  /// @param delayer __const std::unique_ptr<rtos::IDelayer>&__ The delayer used
  /// @return __PIDTrajectoryFollowerBuilder*__ Pointer to the current builder
  PIDTrajectoryFollowerBuilder* withDelayer(
      const std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Sets the mutex used by the trajectory follower
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex used
  /// @return __PIDTrajectoryFollowerBuilder*__ Pointer to the current builder
  PIDTrajectoryFollowerBuilder* withMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Sets the task used by the trajectory follower
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task used
  /// @return __PIDTrajectoryFollowerBuilder*__ Pointer to the current builder
  PIDTrajectoryFollowerBuilder* withTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Sets the clock used by the trajectory follower
  /// @param clock __const std::unique_ptr<rtos::IClock>&__ The clock used
  /// @return __PIDTrajectoryFollowerBuilder*__ Pointer to the current builder
  PIDTrajectoryFollowerBuilder* withClock(
      const std::unique_ptr<rtos::IClock>& clock);

  /// @brief Sets the x-axis PID controller used by the trajectory follower
  /// @param x_pid __PID&__ The PID controller to use
  /// @return __PIDTrajectoryFollowerBuilder*__ Pointer to the current builder
  PIDTrajectoryFollowerBuilder* withXPID(PID& x_pid);

  /// @brief Sets the y-axis PID controller used by the trajectory follower
  /// @param y_pid __PID&__ The PID controller to use
  /// @return __PIDTrajectoryFollowerBuilder*__ Pointer to the current builder
  PIDTrajectoryFollowerBuilder* withYPID(PID& y_pid);

  /// @brief Sets the angular PID controller used by the trajectory follower
  /// @param theta_pid __PID&__ The PID controller to use
  /// @return __PIDTrajectoryFollowerBuilder*__ Pointer to the current builder
  PIDTrajectoryFollowerBuilder* withThetaPID(PID& theta_pid);

  /// @brief Sets the target distance tolerance used by the trajectory follower
  /// @param target_tolerance __double__ The target distance tolerance
  /// @return __PIDTrajectoryFollowerBuilder*__ Pointer to the current builder
  PIDTrajectoryFollowerBuilder* withTargetTolerance(double target_tolerance);

  /// @brief Sets the target velocity tolerance used by the trajectory follower
  /// @param target_velocity __double__ The target velocity tolerance
  /// @return __PIDTrajectoryFollowerBuilder*__ Pointer to the current builder
  PIDTrajectoryFollowerBuilder* withTargetVelocity(double target_velocity);

  /// @brief Builds a PIDTrajectoryFollower object
  /// @return __std::unique_ptr<PIDTrajectoryFollower>__ The new
  /// PIDTrajectoryFollower
  std::unique_ptr<PIDTrajectoryFollower> build();
};
}  // namespace trajectory_follower
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif