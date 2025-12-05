#include "driftless/control/trajectory/trajectory_generator/kinematics/MaxVelocityConstraintBuilder.hpp"

namespace driftless::control::trajectory::trajectory_generator::kinematics {
  MaxVelocityConstraintBuilder* MaxVelocityConstraintBuilder::withMaxVelocity(double max_velocity) {
    m_max_velocity = max_velocity;
    return this;
  }

  std::unique_ptr<IKinematicConstraint> MaxVelocityConstraintBuilder::build() {
    std::unique_ptr<MaxVelocityConstraint> constraint{std::make_unique<MaxVelocityConstraint>()};
    constraint->setMaxVelocity(m_max_velocity);

    return constraint;
  }
}