#include "driftless/control/trajectory/TrajectoryPoint.hpp"

namespace driftless::control::trajectory {
TrajectoryPoint::TrajectoryPoint(double x, double y) : m_x{x}, m_y{y} {}

TrajectoryPoint::TrajectoryPoint(double x, double y, double heading)
    : m_x{x}, m_y{y}, m_heading{heading} {}

TrajectoryPoint::TrajectoryPoint(double x, double y, double heading,
                                 double velocity, double angular_velocity)
    : m_x{x},
      m_y{y},
      m_heading{heading},
      m_velocity{velocity},
      m_angular_velocity{angular_velocity} {}

double TrajectoryPoint::distanceTo(const TrajectoryPoint& other) const {
  return std::sqrt(std::pow(other.m_x - m_x, 2) + std::pow(other.m_y - m_y, 2));
}

double TrajectoryPoint::angleTo(const TrajectoryPoint& other) const {
  return std::atan2(other.m_y - m_y, other.m_x - m_x);
}

}  // namespace driftless::control::trajectory