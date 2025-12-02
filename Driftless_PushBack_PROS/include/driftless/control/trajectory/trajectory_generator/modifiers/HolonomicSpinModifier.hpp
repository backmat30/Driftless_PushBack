#ifndef __HOLONOMIC_SPIN_MODIFIER_HPP__
#define __HOLONOMIC_SPIN_MODIFIER_HPP__

#include <algorithm>
#include <vector>

#include "driftless/control/trajectory/SpinContext.hpp"
#include "driftless/control/trajectory/trajectory_generator/modifiers/ITrajectoryModifier.hpp"
#include "driftless/utils/UtilityFunctions.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for trajectory generation and following
/// @author Matthew Backman
namespace trajectory {

/// @brief namespace for building trajectories
/// @author Matthew Backman
namespace trajectory_generator {

/// @brief Namespace for trajectory modifiers
/// @author Matthew Backman
namespace modifiers {

/// @brief Trajectory modifier to apply spins along the path for a holonomic
/// drive train
/// @author Matthew Backman
class HolonomicSpinModifier : public ITrajectoryModifier {
 private:
  /// @brief The maximum angular acceleration while spinning
  double m_max_acceleration{};

  /// @brief The heading of the robot at the start of the trajectory
  double m_starting_heading{};

  /// @brief The list of spins to apply to a trajectory
  std::vector<SpinContext> m_spins{};

 public:
  /// @brief Applies the modifier to the provided trajectory
  /// @param trajectory __std::vector<TrajectoryPoint>&__ The trajectory to
  /// modify
  void applyModifier(std::vector<TrajectoryPoint>& trajectory) override;

  /// @brief Sets the spins to use
  /// @param spins __std::vector<SpinContext>&__ The spins to use
  void setSpins(std::vector<SpinContext>& spins);

  /// @brief Sets the max acceleration for spins
  /// @param max_acceleration __double__ The max acceleration to use
  void setMaxAcceleration(double max_acceleration);

  /// @brief Sets the starting heading of the trajectory
  /// @param starting_heading __double__ The starting heading
  void setStartingHeading(double starting_heading);
};
}  // namespace modifiers
}  // namespace trajectory_generator
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif