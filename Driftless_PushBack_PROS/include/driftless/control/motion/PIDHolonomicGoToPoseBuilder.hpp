#ifndef __PID_HOLONOMIC_GO_TO_POSE_BUILDER_HPP__
#define __PID_HOLONOMIC_GO_TO_POSE_BUILDER_HPP__

#include "driftless/control/motion/PIDHolonomicGoToPose.hpp"

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
class PIDHolonomicGoToPoseBuilder {
 private:
  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

  PID m_x_pid{};

  PID m_y_pid{};

  PID m_rotational_pid{};

  double m_distance_tolerance{};

  double m_velocity_tolerance{};

  double m_angular_tolerance{};

 public:
  /// @brief Sets the delayer for the control
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer to use
  /// @return __PIDHolonomicGoToPoseBuilder*__ Pointer to the builder
  PIDHolonomicGoToPoseBuilder* withDelayer(
      std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Sets the mutex for the control
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex to use
  /// @return __PIDHolonomicGoToPoseBuilder*__ Pointer to the builder
  PIDHolonomicGoToPoseBuilder* withMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Sets the task for the control
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task to use
  /// @return __PIDHolonomicGoToPoseBuilder*__ Pointer to the builder
  PIDHolonomicGoToPoseBuilder* withTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Sets the x PID for the control
  /// @param pid __PID&__ The x PID to use
  /// @return __PIDHolonomicGoToPoseBuilder*__ Pointer to the builder
  PIDHolonomicGoToPoseBuilder* withXPID(const PID& pid);

  /// @brief Sets the y PID for the control
  /// @param pid __PID&__ The y PID to use
  /// @return __PIDHolonomicGoToPoseBuilder*__ Pointer to the builder
  PIDHolonomicGoToPoseBuilder* withYPID(const PID& pid);

  /// @brief Sets the rotational PID for the control
  /// @param pid __PID&__ The rotational PID to use
  /// @return __PIDHolonomicGoToPoseBuilder*__ Pointer to the builder
  PIDHolonomicGoToPoseBuilder* withRotationalPID(const PID& pid);

  /// @brief Sets the distance tolerance for the control
  /// @param tolerance __double__ The distance tolerance to use
  /// @return __PIDHolonomicGoToPoseBuilder*__ Pointer to the builder
  PIDHolonomicGoToPoseBuilder* withDistanceTolerance(double tolerance);

  /// @brief Sets the velocity tolerance for the control
  /// @param tolerance __double__ The velocity tolerance to use
  /// @return __PIDHolonomicGoToPoseBuilder*__ Pointer to the builder
  PIDHolonomicGoToPoseBuilder* withVelocityTolerance(double tolerance);

  /// @brief Sets the angular tolerance for the control
  /// @param tolerance __double__ The angular tolerance to use
  /// @return __PIDHolonomicGoToPoseBuilder*__ Pointer to the builder
  PIDHolonomicGoToPoseBuilder* withAngularTolerance(double tolerance);

  /// @brief Builds a PIDHolonomicGoToPoint object
  /// @return __std::unique_ptr<PIDHolonomicGoToPose>__ The built object
  std::unique_ptr<PIDHolonomicGoToPose> build();
};
}  // namespace motion
}  // namespace control
}  // namespace driftless
#endif