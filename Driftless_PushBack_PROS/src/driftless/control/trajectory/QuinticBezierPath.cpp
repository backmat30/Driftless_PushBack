#include "driftless/control/trajectory/QuinticBezierPath.hpp"

namespace driftless::control::trajectory {
QuinticBezierPath::QuinticBezierPath(Point start, Point c0, Point c1, Point c2,
                                     Point c3, Point end) {
  m_control_points << start, c0, c1, c2, c3, end;

  // Calculate coefficients for the bezier curve
  m_coefficients << 1, -5, 10, -10, 5, -1, 0, 5, -20, 30, -20, 5, 0, 0, 10, -30,
      30, -10, 0, 0, 0, 10, -20, 10, 0, 0, 0, 0, 5, -5, 0, 0, 0, 0, 0, 1;

  // Calculate coefficients for the first derivative of the bezier curve
  m_first_derivative_coefficients << 5, -25, 50, -50, 25, -5, 0, 20, -80, 120, -80,
      20, 0, 0, 30, -90, 90, -30, 0, 0, 0, 20, -40, 20, 0, 0, 0, 0, 5, -5;

  // Calculate coefficients for the second derivative of the bezier curve
  m_second_derivative_coefficients << 20, -100, 200, -200, 100, -20, 0, 60, -240,
      360, -240, 60, 0, 0, 60, -180, 180, -60, 0, 0, 0, 20, -40, 20;
}

Point QuinticBezierPath::getPoint(double t) const {
  Eigen::Matrix<double, 1, 6> T;
  T << std::pow(t, 5), std::pow(t, 4), std::pow(t, 3), std::pow(t, 2), t, 1;
  auto result = (T * m_coefficients) * m_control_points;
  return result(0);
}

Point QuinticBezierPath::getDerivative(double t) const {
  Eigen::Matrix<double, 1, 5> T;
  T << std::pow(t, 4), std::pow(t, 3), std::pow(t, 2), t, 1;
  auto result = (T * m_first_derivative_coefficients) * m_control_points;
  return result(0);
}

Point QuinticBezierPath::getSecondDerivative(double t) const {
  Eigen::Matrix<double, 1, 4> T;
  T << std::pow(t, 3), std::pow(t, 2), t, 1;
  auto result = (T * m_second_derivative_coefficients) * m_control_points;
  return result(0);
}

double QuinticBezierPath::getMaxTime() const { return 1.0; }
}  // namespace driftless::control::trajectory