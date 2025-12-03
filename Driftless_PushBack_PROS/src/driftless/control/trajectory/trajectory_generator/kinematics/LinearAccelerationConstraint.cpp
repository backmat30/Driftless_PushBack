#include "driftless/control/trajectory/trajectory_generator/kinematics/LinearAccelerationConstraint.hpp"

namespace driftless::control::trajectory::trajectory_generator::kinematics {
double LinearAccelerationConstraint::getMaxVelocity(
    std::unique_ptr<IPath>& path, TrajectoryPoint last_point, double delta_d,
    double t) {
  double max_velocity = std::sqrt(std::pow(last_point.m_velocity, 2) +
                                  2 * m_max_acceleration * delta_d);
  return max_velocity;
}

void LinearAccelerationConstraint::setMaxAcceleration(double max_acceleration) {
  m_max_acceleration = max_acceleration;
}
}  // namespace driftless::control::trajectory::trajectory_generator::kinematics