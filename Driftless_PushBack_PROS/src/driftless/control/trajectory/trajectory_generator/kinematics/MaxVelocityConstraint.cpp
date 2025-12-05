#include "driftless/control/trajectory/trajectory_generator/kinematics/MaxVelocityConstraint.hpp"

namespace driftless::control::trajectory::trajectory_generator::kinematics {
double MaxVelocityConstraint::getMaxVelocity(std::unique_ptr<IPath>& path,
                                             TrajectoryPoint last_point,
                                             double delta_d, double t) {
  return m_max_velocity;
}

void MaxVelocityConstraint::setMaxVelocity(double max_velocity) {
  m_max_velocity = max_velocity;
}
}  // namespace driftless::control::trajectory::trajectory_generator::kinematics