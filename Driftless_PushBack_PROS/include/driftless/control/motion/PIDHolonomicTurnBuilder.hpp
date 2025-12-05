#ifndef __PID_HOLONOMIC_TURN_BUILDER_HPP__
#define __PID_HOLONOMIC_TURN_BUILDER_HPP__

#include "driftless/control/motion/PIDHolonomicTurn.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for direct motion control
/// @author Matthew Backman
namespace motion {

/// @brief Builder class for PIDHolonomicTurn objects
/// @author Matthew Backman
class PIDHolonomicTurnBuilder {
 private:
  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

  PID m_rotational_pid{};

  double m_target_tolerance{};

  double m_target_velocity{};

 public:
  /// @brief Adds a delayer to the builder
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer to add
  /// @return __PIDHolonomicTurnBuilder*__ Reference to the current builder
  PIDHolonomicTurnBuilder* withDelayer(
      const std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Adds a mutex to the builder
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex to add
  /// @return __PIDHolonomicTurnBuilder*__ Reference to the current builder
  PIDHolonomicTurnBuilder* withMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Adds a task to the builder
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task to add
  /// @return __PIDHolonomicTurnBuilder*__ Reference to the current builder
  PIDHolonomicTurnBuilder* withTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Adds a rotational PID to the builder
  /// @param rotational_pid __PID__ The rotational PID to add
  /// @return __PIDHolonomicTurnBuilder*__ Reference to the current builder
  PIDHolonomicTurnBuilder* withRotationalPID(PID rotational_pid);

  /// @brief Adds a target tolerance to the builder
  /// @param target_tolerance __double__ The target tolerance to add
  /// @return __PIDHolonomicTurnBuilder*__ Reference to the current builder
  PIDHolonomicTurnBuilder* withTargetTolerance(double target_tolerance);

  /// @brief Adds a target velocity to the builder
  /// @param target_velocity __double__ The target velocity to add
  /// @return __PIDHolonomicTurnBuilder*__ Reference to the current builder
  PIDHolonomicTurnBuilder* withTargetVelocity(double target_velocity);

  /// @brief Builds a new PIDHolonomicTurn object
  /// @return __std::unique_ptr<PIDHolonomicTurn>&__ The newly created
  /// PIDHolonomicTurn
  std::unique_ptr<PIDHolonomicTurn> build();
};
}  // namespace motion
}  // namespace control
}  // namespace driftless

#endif