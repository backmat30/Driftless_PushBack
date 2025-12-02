#include "driftless/control/trajectory/trajectory_generator/kinematics/CentripetalAccelerationConstraint.hpp"

namespace driftless::control::trajectory::trajectory_generator::kinematics {
double CentripetalAccelerationConstraint::getMaxVelocity(
    std::unique_ptr<IPath>& path, TrajectoryPoint last_point, double delta_d,
    double t) {
  Point current_point{path->getPoint(t)};
  Point derivative{path->getDerivative(t)};
  Point second_derivative{path->getSecondDerivative(t)};

  double curvature{
      (derivative.getX() * second_derivative.getY() -
       derivative.getY() * second_derivative.getX()) /
      (std::pow(std::pow(derivative.getX(), 2) + std::pow(derivative.getY(), 2),
                1.5))};

  double max_velocity{__DBL_MAX__};
  if (std::abs(curvature) > 1E-5) {
    max_velocity =
        std::sqrt(std::abs(m_max_centripetal_acceleration / curvature));
  }

  return max_velocity;
}

void CentripetalAccelerationConstraint::setMaxCentripetalAcceleration(
    double max_centripetal_acceleration) {
  m_max_centripetal_acceleration = max_centripetal_acceleration;
}
}  // namespace driftless::control::trajectory::trajectory_generator::kinematics