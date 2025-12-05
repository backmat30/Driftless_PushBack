#ifndef __HOLONOMIC_SPIN_MODIFIER_BUILDER_HPP__
#define __HOLONOMIC_SPIN_MODIFIER_BUILDER_HPP__

#include <memory>

#include "driftless/control/trajectory/trajectory_generator/modifiers/HolonomicSpinModifier.hpp"

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
class HolonomicSpinModifierBuilder {
 private:
  double m_max_acceleration{};

  double m_starting_heading{};

  std::vector<SpinContext> m_spins{};

 public:
  /// @brief Sets the spins to use
  /// @param spins __std::vector<SpinContext>&__ The spins to use
  HolonomicSpinModifierBuilder* withSpins(std::vector<SpinContext>& spins);

  /// @brief Sets the max acceleration for spins
  /// @param max_acceleration __double__ The max acceleration to use
  HolonomicSpinModifierBuilder* withMaxAcceleration(double max_acceleration);

  /// @brief Sets the starting heading of the trajectory
  /// @param starting_heading __double__ The starting heading
  HolonomicSpinModifierBuilder* withStartingHeading(double starting_heading);

  /// @brief Builds the HolonomicSpinModifier
  /// @return __std::unique_ptr<HolonomicSpinModifier>__ The built modifier
  std::unique_ptr<HolonomicSpinModifier> build();
};
}  // namespace modifiers
}  // namespace trajectory_generator
}  // namespace trajectory
}  // namespace control
}  // namespace driftless

#endif