#include "driftless/control/trajectory/QuinticBezierSplinePath.hpp"

namespace driftless::control::trajectory {
QuinticBezierSplinePath::QuinticBezierSplinePath(
    const std::vector<Point>& control_points) {
  m_segments = std::vector<QuinticBezierPath>{};
  m_max_time = 0.0;
  // if the control set is invalid, return an empty vector
  if (control_points.size() != 0 && (control_points.size() - 1) % 3 == 0) {
    for (int i{}; i < control_points.size() - 1; i += 3) {
      // add a new curve to the list with empty points in the unknown positions
      m_segments.push_back(QuinticBezierPath{
          control_points[i], Point{}, control_points[i + 1],
          control_points[i + 2], Point{}, control_points[i + 3]});
      m_max_time++;
    }

    // calculate first smoothing point for the first curve
    m_segments[0].setControlPoint(1, (m_segments[0].getControlPoint(0) +
                                      m_segments[0].getControlPoint(2)) /
                                         2.0);

    // calculate first smoothing point for other curves
    for (int i{1}; i < m_segments.size(); ++i) {
      m_segments[i].setControlPoint(1,
                                    ((m_segments[i].getControlPoint(0) * 4.0) -
                                     m_segments[i - 1].getControlPoint(3) +
                                     m_segments[i].getControlPoint(2)) /
                                        4.0);
    }
    // calculate last smoothing point for curves other than last
    for (int i{}; i < m_segments.size() - 1; ++i) {
      m_segments[i].setControlPoint(4,
                                    m_segments[i].getControlPoint(5) * 2.0 -
                                        m_segments[i + 1].getControlPoint(1));
    }
    // calculate last smoothing point for last curve
    m_segments[m_segments.size() - 1].setControlPoint(
        4, (m_segments[m_segments.size() - 1].getControlPoint(5) +
            m_segments[m_segments.size() - 1].getControlPoint(3)) /
               2.0);
  }
}

Point QuinticBezierSplinePath::getPoint(double t) const {
  Point result{};
  if (t >= 0 && t < m_max_time) {
    int segment = static_cast<int>(t);
    double local_t = t - segment;
    result = m_segments[segment].getPoint(local_t);
  }
  return result;
}

Point QuinticBezierSplinePath::getDerivative(double t) const {
  Point result{};
  if (t >= 0 && t < m_max_time) {
    int segment = static_cast<int>(t);
    double local_t = t - segment;
    result = m_segments[segment].getDerivative(local_t);
  }
  return result;
}

Point QuinticBezierSplinePath::getSecondDerivative(double t) const {
  Point result{};
  if (t >= 0 && t < m_max_time) {
    int segment = static_cast<int>(t);
    double local_t = t - segment;
    result = m_segments[segment].getSecondDerivative(local_t);
  }
  return result;
}

double QuinticBezierSplinePath::getMaxTime() const { return m_max_time; }
}  // namespace driftless::control::trajectory