#ifndef __PID_HOLONOMIC_GO_TO_POINT_BUILDER_HPP__
#define __PID_HOLONOMIC_GO_TO_POINT_BUILDER_HPP__

#include "driftless/control/motion/PIDHolonomicGoToPoint.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for direct motion control
/// @author Matthew Backman
namespace motion {

/// @brief Builder class for PIDHolonomicGoToPoint objects
class PIDHolonomicGoToPointBuilder {
 private:
  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

  PID m_x_pid{};

  PID m_y_pid{};

  double m_distance_tolerance{};

  double m_velocity_tolerance{};

 public:
  /// @brief Sets the delayer for the control
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer to use
  /// @return __PIDHolonomicGoToPointBuilder*__ Pointer to the builder
  PIDHolonomicGoToPointBuilder* withDelayer(
      std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Sets the mutex for the control
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex to use
  /// @return __PIDHolonomicGoToPointBuilder*__ Pointer to the builder
  PIDHolonomicGoToPointBuilder* withMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Sets the task for the control
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task to use
  /// @return __PIDHolonomicGoToPointBuilder*__ Pointer to the builder
  PIDHolonomicGoToPointBuilder* withTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Sets the x PID for the control
  /// @param pid __PID&__ The x PID to use
  /// @return __PIDHolonomicGoToPointBuilder*__ Pointer to the builder
  PIDHolonomicGoToPointBuilder* withXPID(const PID& pid);

  /// @brief Sets the y PID for the control
  /// @param pid __PID&__ The y PID to use
  /// @return __PIDHolonomicGoToPointBuilder*__ Pointer to the builder
  PIDHolonomicGoToPointBuilder* withYPID(const PID& pid);

  /// @brief Sets the distance tolerance for the control
  /// @param tolerance __double__ The distance tolerance to use
  /// @return __PIDHolonomicGoToPointBuilder*__ Pointer to the builder
  PIDHolonomicGoToPointBuilder* withDistanceTolerance(double tolerance);

  /// @brief Sets the velocity tolerance for the control
  /// @param tolerance __double__ The velocity tolerance to use
  /// @return __PIDHolonomicGoToPointBuilder*__ Pointer to the builder
  PIDHolonomicGoToPointBuilder* withVelocityTolerance(double tolerance);

  /// @brief Builds a PIDHolonomicGoToPoint object
  /// @return __std::unique_ptr<PIDHolonomicGoToPoint>__ The built object
  std::unique_ptr<PIDHolonomicGoToPoint> build();
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif