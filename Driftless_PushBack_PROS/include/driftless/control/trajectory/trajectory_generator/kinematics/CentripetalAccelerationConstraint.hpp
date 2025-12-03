#ifndef __CENTRIPETAL_ACCELERATION_CONSTRAINT_HPP__
#define __CENTRIPETAL_ACCELERATION_CONSTRAINT_HPP__

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
/// @brief Kinematic constraint to limit the centripetal acceleration of the
/// robot
class CentripetalAccelerationConstraint : public IKinematicConstraint {
 private:
  /// @brief The max centripetal acceleration of the robot in inches per
  /// second^2
  double m_max_centripetal_acceleration;

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

  /// @brief Sets the max centripetal acceleration of the robot
  /// @param max_centripetal_acceleration __double__ The max centripetal
  /// acceleration in inches per second^2
  void setMaxCentripetalAcceleration(double max_centripetal_acceleration);
};
}  // namespace kinematics
}  // namespace trajectory_generator
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif