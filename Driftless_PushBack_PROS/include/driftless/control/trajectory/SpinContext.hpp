#ifndef __SPIN_CONTEXT_HPP__
#define __SPIN_CONTEXT_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for trajectory generation and following
/// @author Matthew Backman
namespace trajectory {

/// @brief Provides context for a spin for a holonomic drive train
struct SpinContext {
  /// @brief The t-value to start turning at
  double start_t{};

  /// @brief The t-value to be done turning at
  double end_t{};

  /// @brief The desired angle at the end of the spin
  double end_angle{};
};
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif