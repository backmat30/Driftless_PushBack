#ifndef __CENTRIPETAL_ACCELERATION_CONSTRAINT_BUILDER_HPP__
#define __CENTRIPETAL_ACCELERATION_CONSTRAINT_BUILDER_HPP__

#include <memory>

#include "driftless/control/trajectory/trajectory_generator/kinematics/CentripetalAccelerationConstraint.hpp"

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

/// @brief Builder class for CentripetalAccelerationConstraint objects
/// @author Matthew Backman
class CentripetalAccelerationConstraintBuilder {
 private:
  /// @brief The max centripetal acceleration of the robot in inches per
  /// second^2
  double m_max_centripetal_acceleration{};

 public:
  /// @brief Set the maximum centripetal acceleration of the robot
  /// @param max_centripetal_acceleration __double__ The max centripetal
  /// acceleration in inches per second^2
  /// @return __CentripetalAccelerationConstraintBuilder*__ Pointer to this
  /// builder
  CentripetalAccelerationConstraintBuilder* withMaxCentripetalAcceleration(
      double max_centripetal_acceleration);

  /// @brief Builds the CentripetalAccelerationConstraint object
  /// @return __std::unique_ptr<IKinematicConstraint>__ The built constraint
  std::unique_ptr<IKinematicConstraint> build();
};
}  // namespace kinematics
}  // namespace trajectory_generator
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif