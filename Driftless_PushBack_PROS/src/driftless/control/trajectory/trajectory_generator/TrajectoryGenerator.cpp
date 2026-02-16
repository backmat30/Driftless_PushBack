#include "driftless/control/trajectory/trajectory_generator/TrajectoryGenerator.hpp"

namespace driftless::control::trajectory::trajectory_generator {
TrajectoryGenerator::TrajectoryGenerator(double delta_d) : m_delta_d{delta_d} {}

void TrajectoryGenerator::addConstraint(
    std::unique_ptr<kinematics::IKinematicConstraint>& constraint) {
  m_constraints.push_back(std::move(constraint));
}

void TrajectoryGenerator::generateTrajectory(std::unique_ptr<IPath>& path) {
  double t = 0;
  std::vector<TrajectoryPoint> dist_trajectory{};
  dist_trajectory.push_back(TrajectoryPoint{
      path->getPoint(0).getX(), path->getPoint(0).getY(),
      std::atan2(path->getDerivative(0).getY(), path->getDerivative(0).getX()),
      0, 0});
  TrajectoryPoint last_point{dist_trajectory.back()};

  // Forward pass
  while (t < path->getMaxTime()) {
    double max_velocity{__DBL_MAX__};

    // apply constraints
    for (auto& constraint : m_constraints) {
      max_velocity =
          std::min(constraint->getMaxVelocity(path, last_point, m_delta_d, t),
                   max_velocity);
    }

    Point derivative{path->getDerivative(t)};
    Point second_derivative{path->getSecondDerivative(t)};

    // find the change in time needed to satisfy the change in distance
    double dt{m_delta_d / (std::sqrt(std::pow(derivative.getX(), 2) +
                                     std::pow(derivative.getY(), 2)))};

    // find the curvature at the current point
    double curvature{(derivative.getX() * second_derivative.getY() -
                      derivative.getY() * second_derivative.getX()) /
                     (std::pow(std::pow(derivative.getX(), 2) +
                                   std::pow(derivative.getY(), 2),
                               1.5))};

    last_point =
        TrajectoryPoint{path->getPoint(t).getX(), path->getPoint(t).getY(),
                        std::atan2(derivative.getY(), derivative.getX()),
                        max_velocity, max_velocity * curvature};

    dist_trajectory.push_back(last_point);
    t += dt;
  }

  int i = dist_trajectory.size() - 1;
  t = path->getMaxTime();
  last_point = TrajectoryPoint{
      path->getPoint(t).getX(), path->getPoint(t).getY(),
      std::atan2(path->getDerivative(t).getY(), path->getDerivative(t).getX()),
      0, 0};

  // backwards pass
  while (t > 0) {
    // only re-apply constraints if the previous velocity was changed
    if (last_point.m_velocity < dist_trajectory[i].m_velocity) {
      dist_trajectory[i] = last_point;
    }

    double max_velocity{__DBL_MAX__};
    for (auto& constraint : m_constraints) {
      max_velocity =
          std::min(constraint->getMaxVelocity(path, last_point, m_delta_d, t),
                   max_velocity);
    }

    Point derivative{path->getDerivative(t)};
    Point second_derivative{path->getSecondDerivative(t)};

    double dt{m_delta_d / (std::sqrt(std::pow(derivative.getX(), 2) +
                                     std::pow(derivative.getY(), 2)))};

    double curvature{(derivative.getX() * second_derivative.getY() -
                      derivative.getY() * second_derivative.getX()) /
                     (std::pow(std::pow(derivative.getX(), 2) +
                                   std::pow(derivative.getY(), 2),
                               1.5))};

    last_point =
        TrajectoryPoint{path->getPoint(t).getX(), path->getPoint(t).getY(),
                        atan2(derivative.getY(), derivative.getX()),
                        max_velocity, max_velocity * curvature};

    t -= dt;
    i--;
  }

  // Create m_trajectory as a time based trajectory, using dist_trajectory
  double current_distance{};
  double delta_t = 0.02;
  double current_time{};
  while (static_cast<int>(current_distance / m_delta_d) <
         dist_trajectory.size()) {
    TrajectoryPoint point =
        dist_trajectory[static_cast<int>(current_distance / m_delta_d)];
    if (point.m_velocity != 0) {
      current_distance += point.m_velocity * delta_t;
    } else {
      current_distance += m_delta_d;
    }
    // get x and y velocities
    point.m_x_velocity = point.m_velocity * std::cos(point.m_heading);
    point.m_y_velocity = point.m_velocity * std::sin(point.m_heading);
    m_trajectory.push_back(point);
    current_time += delta_t;
  }
}

std::vector<TrajectoryPoint> TrajectoryGenerator::getTrajectory() {
  return m_trajectory;
}

}  // namespace driftless::control::trajectory::trajectory_generator