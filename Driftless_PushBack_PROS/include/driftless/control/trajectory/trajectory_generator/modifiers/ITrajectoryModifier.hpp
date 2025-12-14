#ifndef __I_TRAJECTORY_MODIFIER_HPP__
#define __I_TRAJECTORY_MODIFIER_HPP__

#include <vector>

#include "driftless/control/trajectory/TrajectoryPoint.hpp"

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

/// @brief Namespace for trajectory modifiers
/// @author Matthew Backman
namespace modifiers {

/// @brief Interface for a generic trajectory modifier
/// @author Matthew Backman
class ITrajectoryModifier {
 public:
  /// @brief Destroys the trajectory modifier
  virtual ~ITrajectoryModifier() = default;

  /// @brief Applies the modifier to the provided trajectory
  /// @param trajectory __std::vector<TrajectoryPoint>&__ The trajectory to
  /// modify
  virtual void applyModifier(std::vector<TrajectoryPoint>& trajectory) = 0;
};
}  // namespace modifiers
}  // namespace trajectory_generator
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif