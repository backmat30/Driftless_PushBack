#ifndef __I_KINEMATIC_CONSTRAINT_HPP__
#define __I_KINEMATIC_CONSTRAINT_HPP__

#include <cstdarg>
#include <memory>

#include "driftless/control/trajectory/IPath.hpp"
#include "driftless/control/trajectory/TrajectoryPoint.hpp"

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

/// @brief Class representing a generic kinematic constraint
/// @author Matthew Backman
class IKinematicConstraint {
 public:
  /// @brief Destroys the kinematic constraint
  virtual ~IKinematicConstraint() = default;

  /// @brief Gets the max velocity at a given point along the path
  /// @param path __std::unique_ptr<IPath>&__ The path being used for the
  /// trajectory
  /// @param last_point __TrajectoryPoint__ The previous point in the trajectory
  /// @param delta_d __double__ The change in distance between points
  /// @param t __double__ The time value along the path
  /// @return __double__ The max velocity at the given point
  virtual double getMaxVelocity(std::unique_ptr<IPath>& path,
                                TrajectoryPoint last_point, double delta_d,
                                double t) = 0;
};
}  // namespace kinematics
}  // namespace trajectory_generator
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif