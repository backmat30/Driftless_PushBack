#ifndef __QUINTIC_BEZIER_SPLINE_PATH_HPP__
#define __QUINTIC_BEZIER_SPLINE_PATH_HPP__

#include "driftless/control/trajectory/IPath.hpp"
#include "driftless/control/trajectory/QuinticBezierPath.hpp"

namespace driftless {
namespace control {
namespace trajectory {
class QuinticBezierSplinePath : public IPath {
 private:
  /// @brief The bezier curves within the spline
  std::vector<QuinticBezierPath> m_segments;

  /// @brief The maximum time of the spline
  double m_max_time;

 public:
  /// @brief Constructs a new quintic bezier spline path
  /// @param control_points __const std::vector<Point>&__ The control points for
  /// the spline
  QuinticBezierSplinePath(const std::vector<Point>& control_points);

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
};
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif