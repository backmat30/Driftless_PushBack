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
class MaxVelocityConstraint : public IKinematicConstraint {
 private:
  double m_max_velocity;

 public:
  double getMaxVelocity(std::unique_ptr<IPath>& path,
                        TrajectoryPoint last_point, double delta_d,
                        double t) override;

  void setMaxVelocity(double max_velocity);
};
}  // namespace kinematics
}  // namespace trajectory_generator
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif