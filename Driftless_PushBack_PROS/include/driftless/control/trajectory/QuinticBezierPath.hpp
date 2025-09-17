#ifndef __QUINTIC_BEZIER_PATH_HPP__
#define __QUINTIC_BEZIER_PATH_HPP__

#include <Eigen/Dense>

#include "driftless/control/trajectory/IPath.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for trajectory generation and following
/// @author Matthew Backman
namespace trajectory {

/// @brief Class representing a quintic bezier path
/// @author Matthew Backman
class QuinticBezierPath : public IPath {
 private:
  /// @brief Control points for the bezier curve
  Eigen::Matrix<double, 6, 2> m_control_points;

  /// @brief Coefficients for the bezier curve
  Eigen::Matrix<double, 6, 6> m_coefficients;

  /// @brief Coefficients for the first derivative of the bezier curve
  Eigen::Matrix<double, 5, 6> m_first_derivative_coefficients;

  /// @brief Coefficients for the second derivative of the bezier curve
  Eigen::Matrix<double, 4, 6> m_second_derivative_coefficients;

 public:
  /// @brief Constructs a new quintic bezier path
  QuinticBezierPath(Point start, Point c0, Point c1, Point c2, Point c3,
                    Point end);

  /// @brief Gets the point at time t
  /// @param t __double__ The time
  /// @return __Point__ The point at time t
  Point getPoint(double t) const override;

  /// @brief Gets the derivative at time t
  /// @param t __double__ The time
  /// @return __Point__ The derivative at time t
  Point getDerivative(double t) const override;

  /// @brief Gets the second derivative at time t
  /// @param t __double__ The time
  /// @return __Point__ The second derivative at time t
  Point getSecondDerivative(double t) const override;

  /// @brief Gets the maximum time
  /// @return __double__ The maximum time
  double getMaxTime() const override;

  /// @brief Sets a control point at the given index
  /// @param index __int__ The index of the control point
  /// @param point __const Point__ The new control point
  void setControlPoint(int index, Point point);

  Point getControlPoint(int index) const;
};
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif