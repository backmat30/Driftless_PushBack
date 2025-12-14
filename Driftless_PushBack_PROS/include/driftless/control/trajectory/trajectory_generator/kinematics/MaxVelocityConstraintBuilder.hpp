#ifndef __MAX_VELOCITY_CONSTRAINT_BUILDER_HPP__
#define __MAX_VELOCITY_CONSTRAINT_BUILDER_HPP__

#include <memory>

#include "driftless/control/trajectory/trajectory_generator/kinematics/IKinematicConstraint.hpp"
#include "driftless/control/trajectory/trajectory_generator/kinematics/MaxVelocityConstraint.hpp"

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

/// @brief Builder for MaxVelocityConstraint objects
/// @author Matthew Backman
class MaxVelocityConstraintBuilder {
 private:
  /// @brief The max velocity of the robot in inches per second
  double m_max_velocity{};

 public:
  /// @brief Set the max velocity of the robot in inches per second
  /// @param max_velocity __double__ The max velocity in inches per second
  /// @return __MaxVelocityConstraintBuilder*__ Pointer to this builder
  MaxVelocityConstraintBuilder* withMaxVelocity(double max_velocity);

  /// @brief Builds the MaxVelocityConstraint object
  /// @return __std::unique_ptr<IKinematicConstraint>__ The built constraint
  std::unique_ptr<IKinematicConstraint> build();
};
}  // namespace kinematics
}  // namespace trajectory_generator
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif