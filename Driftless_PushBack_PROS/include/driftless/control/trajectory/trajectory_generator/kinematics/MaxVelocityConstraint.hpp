#ifndef __MAX_VELOCITY_CONSTRAINT_HPP__
#define __MAX_VELOCITY_CONSTRAINT_HPP__

#include "driftless/control/trajectory/trajectory_generator/kinematics/IKinematicConstraint.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for trajectory generation and following
/// @author Matthew Backman
namespace trajectory {

/// @brief Namespace for building trajectories
/// @author Matthew Backman
namespace trajectory_generator {

/// @brief Namespace for kinematic constraints used in building trajectories
/// @author Matthew Backman
namespace kinematics {

/// @brief Kinematic constraint to limit the max velocity of the robot
/// @author Matthew Backman
class MaxVelocityConstraint : public IKinematicConstraint {
 private:
  /// @brief The max velocity of the robot in inches per second
  double m_max_velocity;

 public:
  /// @brief Gets the max velocity at a given point along the path
  /// @param path __std::unique_ptr<IPath>&__ The path being used for the
  /// trajectory
  /// @param last_point __TrajectoryPoint__ The previous point in the trajectory
  /// @param delta_d __double__ The change in distance between points
  /// @param t __double__ The time value along the path
  /// @return __double__ The max velocity at the given point
  double getMaxVelocity(std::unique_ptr<IPath>& path,
                        TrajectoryPoint last_point, double delta_d,
                        double t) override;

  /// @brief Sets the max velocity of the robot
  /// @param max_velocity __double__ The max velocity in inches per second
  void setMaxVelocity(double max_velocity);
};
}  // namespace kinematics
}  // namespace trajectory_generator
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif