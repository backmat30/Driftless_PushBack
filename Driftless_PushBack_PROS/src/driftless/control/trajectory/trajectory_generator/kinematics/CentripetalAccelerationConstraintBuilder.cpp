#include "driftless/control/trajectory/trajectory_generator/kinematics/CentripetalAccelerationConstraintBuilder.hpp"

namespace driftless::control::trajectory::trajectory_generator::kinematics {
CentripetalAccelerationConstraintBuilder*
CentripetalAccelerationConstraintBuilder::withMaxCentripetalAcceleration(
    double max_centripetal_acceleration) {
  m_max_centripetal_acceleration = max_centripetal_acceleration;
  return this;
}

std::unique_ptr<IKinematicConstraint>
CentripetalAccelerationConstraintBuilder::build() {
  std::unique_ptr<CentripetalAccelerationConstraint> constraint{
      std::make_unique<CentripetalAccelerationConstraint>()};
  constraint->setMaxCentripetalAcceleration(m_max_centripetal_acceleration);
  return constraint;
}
}  // namespace driftless::control::trajectory::trajectory_generator::kinematics