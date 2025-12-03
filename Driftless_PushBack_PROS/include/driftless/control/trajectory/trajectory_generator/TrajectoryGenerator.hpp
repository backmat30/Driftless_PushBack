#ifndef __TRAJECTORY_GENERATOR_HPP__
#define __TRAJECTORY_GENERATOR_HPP__

#include <memory>
#include <vector>

#include "driftless/control/trajectory/IPath.hpp"
#include "driftless/control/trajectory/TrajectoryPoint.hpp"
#include "driftless/control/trajectory/trajectory_generator/kinematics/IKinematicConstraint.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for trajectory generation and following
/// @author Matthew Backman
namespace trajectory {

/// @brief Namespace for building trajectories
/// @author Matthew Backman
namespace trajectory_generator {

/// @brief Generates a trajectory using a provided path and constraints
/// @author Matthew Backman
class TrajectoryGenerator {
 private:
  /// @brief The change in distance between points
  double m_delta_d{};

  /// @brief The set of constraints to be applied
  std::vector<std::unique_ptr<kinematics::IKinematicConstraint>>
      m_constraints{};

  /// @brief The generated trajectory
  std::vector<TrajectoryPoint> m_trajectory{};

 public:
  /// @brief Constructs a new TrajectoryGenerator object
  /// @param delta_d __double__ The change in distance between points
  TrajectoryGenerator(double delta_d);

  /// @brief Adds a constraint to the trajectory
  /// @param constraint __std::unique_ptr<kinematics::IKinematicConstraint>&__
  /// The constraint to add
  void addConstraint(
      std::unique_ptr<kinematics::IKinematicConstraint>& constraint);

  /// @brief Generates a trajectory using the given path
  /// @param path __std::unique_ptr<IPath>&__ The path to use for the trajectory
  void generateTrajectory(std::unique_ptr<IPath>& path);

  /// @brief Gets the latest generated trajectory
  /// @return __std::vector<TrajectoryPoint>__ The generated trajectory
  std::vector<TrajectoryPoint> getTrajectory();
};
}  // namespace trajectory_generator
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif