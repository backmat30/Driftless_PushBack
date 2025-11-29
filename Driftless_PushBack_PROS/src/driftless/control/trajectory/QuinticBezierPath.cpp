#include "driftless/control/trajectory/QuinticBezierPath.hpp"

namespace driftless::control::trajectory {
QuinticBezierPath::QuinticBezierPath(Point start, Point c0, Point c1, Point c2,
                                     Point c3, Point end) {
  m_control_points << start.getX(), start.getY(), c0.getX(), c0.getY(),
      c1.getX(), c1.getY(), c2.getX(), c2.getY(), c3.getX(), c3.getY(),
      end.getX(), end.getY();

  // Calculate coefficients for the bezier curve
  m_coefficients << -1,   5,  -10,  10, -5, 1,
                     5,  -20,  30, -20,  5, 0,
                    -10,  30, -30,  10,  0, 0,
                     10, -20,  10,  0,   0, 0,
                    -5,   5,   0,   0,   0, 0,
                     1,   0,   0,   0,   0, 0;

  // Calculate coefficients for the first derivative of the bezier curve
  m_first_derivative_coefficients << -5,  25, -50,  50, -25, 5,
                                     20, -80, 120, -80, 20,  0,
                                    -30,  90, -90,  30,  0,  0,
                                     20, -40,  20,   0,  0,  0,
                                     -5,   5,   0,   0,  0,  0;

  // Calculate coefficients for the second derivative of the bezier curve
  m_second_derivative_coefficients << -20,  100, -200,  200, -100,  20,
                                       60, -240,  360, -240,   60,   0,
                                      -60,  180, -180,   60,    0,   0,
                                       20,  -40,   20,    0,    0,   0;
}

Point QuinticBezierPath::getPoint(double t) const {
  Eigen::Matrix<double, 1, 6> T;
  T << std::pow(t, 5), std::pow(t, 4), std::pow(t, 3),
      std::pow(t, 2), t, 1;
  auto result = (T * m_coefficients) * m_control_points;
  return Point{result(0), result(1)};
}

Point QuinticBezierPath::getDerivative(double t) const {
  Eigen::Matrix<double, 1, 5> T;
  T << std::pow(t, 4), std::pow(t, 3), std::pow(t, 2), t, 1;
  auto result = (T * m_first_derivative_coefficients) * m_control_points;
  return Point{result(0), result(1)};
}

Point QuinticBezierPath::getSecondDerivative(double t) const {
  Eigen::Matrix<double, 1, 4> T;
  T << std::pow(t, 3), std::pow(t, 2), t, 1;
  auto result = (T * m_second_derivative_coefficients) * m_control_points;
  return Point{result(0), result(1)};
}

double QuinticBezierPath::getMaxTime() const { return 1.0; }

void QuinticBezierPath::setControlPoint(int index, Point point) {
  if (index >= 0 && index < 6) {
    m_control_points(index, 0) = point.getX();
    m_control_points(index, 1) = point.getY();
  }
}

Point QuinticBezierPath::getControlPoint(int index) const {
  Point result{};
  if (index >= 0 && index < 6) {
    result = Point{m_control_points(index, 0), m_control_points(index, 1)};
  }
  return result;
}
}  // namespace driftless::control::trajectory