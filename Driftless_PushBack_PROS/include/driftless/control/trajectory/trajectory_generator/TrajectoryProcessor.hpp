#ifndef __TRAJECTORY_PROCESSOR_HPP__
#define __TRAJECTORY_PROCESSOR_HPP__

#include "driftless/control/trajectory/trajectory_generator/TrajectoryGenerator.hpp"
#include "driftless/control/trajectory/trajectory_generator/modifiers/ITrajectoryModifier.hpp"

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

/// @brief Class to abstract the application of trajectory modifiers
/// @author Matthew Backman
class TrajectoryProcessor {
 public:
 /// @brief Constructs a new Trajectory processor
 TrajectoryProcessor() = default;
  /// @brief Processes a given trajectory using a list of modifiers
  /// @param current_trajectory __const std::vector<TrajectoryPoint>&__ The
  /// trajectory to be modified
  /// @param modifier_list __const
  /// std::vector<std::unique_ptr<modifiers::ITrajectoryModifier>>&__ The modifiers to be applied
  /// @return __std::vector<TrajectoryPoint>__ The trajectory after modifiers
  /// are applied
  std::vector<TrajectoryPoint> process(
      const std::vector<TrajectoryPoint>& current_trajectory,
      const std::vector<std::unique_ptr<modifiers::ITrajectoryModifier>>& modifier_list);
};
}  // namespace trajectory_generator
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif