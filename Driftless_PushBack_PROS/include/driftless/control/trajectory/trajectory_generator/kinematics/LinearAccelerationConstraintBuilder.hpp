#ifndef __LINEAR_ACCELERATION_CONSTRAINT_BUILDER_HPP__
#define __LINEAR_ACCELERATION_CONSTRAINT_BUILDER_HPP__

#include <memory>

#include "driftless/control/trajectory/trajectory_generator/kinematics/LinearAccelerationConstraint.hpp"

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
/// @brief Builder for LinearAccelerationConstraint objects
/// @author Matthew Backman
class LinearAccelerationConstraintBuilder {
 private:
  /// @brief The max acceleration of the robot in inches per second^2
  double m_max_acceleration{};

 public:
  /// @brief Set the maximum acceleration of the robot
  /// @param max_acceleration __double__ The max acceleration in inches per
  /// second^2
  /// @return __LinearAccelerationConstraintBuilder*__ Pointer to this builder
  LinearAccelerationConstraintBuilder* withMaxAcceleration(
      double max_acceleration);

  /// @brief Build the LinearAccelerationConstraint object
  /// @return __std::unique_ptr<IKinematicConstraint>__ The built constraint
  std::unique_ptr<IKinematicConstraint> build();
};
}  // namespace kinematics
}  // namespace trajectory_generator
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif