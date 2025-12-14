#include "driftless/control/trajectory/trajectory_generator/kinematics/LinearAccelerationConstraintBuilder.hpp"

namespace driftless::control::trajectory::trajectory_generator::kinematics {
LinearAccelerationConstraintBuilder*
LinearAccelerationConstraintBuilder::withMaxAcceleration(
    double max_acceleration) {
  m_max_acceleration = max_acceleration;
  return this;
}

std::unique_ptr<IKinematicConstraint>
LinearAccelerationConstraintBuilder::build() {
  std::unique_ptr<LinearAccelerationConstraint> constraint{
      std::make_unique<LinearAccelerationConstraint>()};
  constraint->setMaxAcceleration(m_max_acceleration);
  return constraint;
}
}  // namespace driftless::control::trajectory::trajectory_generator::kinematics