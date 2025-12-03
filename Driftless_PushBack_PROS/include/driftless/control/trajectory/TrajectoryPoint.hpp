#ifndef __TRAJECTORY_POINT_HPP__
#define __TRAJECTORY_POINT_HPP__

#include <cmath>

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for trajectory generation and following
/// @author Matthew Backman
namespace trajectory {

/// @brief Class representing a point in a trajectory
/// @author Matthew Backman
class TrajectoryPoint {
 public:
  /// @brief The x coordinate in inches
  double m_x;

  /// @brief The y coordinate in inches
  double m_y;

  /// @brief The heading in radians
  double m_heading;

  /// @brief The velocity in inches per second
  double m_velocity;

  /// @brief The x velocity in inches per second
  double m_x_velocity;

  /// @brief The y velocity in inches per second
  double m_y_velocity;
  
  /// @brief The angular velocity in radians per second
  double m_angular_velocity;

  /// @brief Constructs a new TrajectoryPoint
  /// @param x __double__ The x coordinate in inches
  /// @param y __double__ The y coordinate in inches
  TrajectoryPoint(double x, double y);

  /// @brief Constructs a new TrajectoryPoint
  /// @param x __double__ The x coordinate in inches
  /// @param y __double__ The y coordinate in inches
  /// @param heading __double__ The heading in radians
  TrajectoryPoint(double x, double y, double heading);

  /// @brief Constructs a new TrajectoryPoint
  /// @param x __double__ The x coordinate in inches
  /// @param y __double__ The y coordinate in inches
  /// @param heading __double__ The heading in radians
  /// @param velocity __double__ The velocity in inches per second
  /// @param angular_velocity __double__ The angular velocity in radians per second
  TrajectoryPoint(double x, double y, double heading, double velocity,
                  double angular_velocity);

  /// @brief Determines the distance to another point
  /// @param other __const TrajectoryPoint&__ The other point
  /// @return __double__ The distance to the point
  double distanceTo(const TrajectoryPoint& other) const;

  /// @brief Determines the angle to another point
  /// @param other __const TrajectoryPoint&__ The other point
  /// @return __double__ The angle to the point
  double angleTo(const TrajectoryPoint& other) const;
};
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif