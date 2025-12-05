#ifndef __E_SUBSYSTEM_STATE_HPP__
#define __E_SUBSYSTEM_STATE_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The enum class for subsystem states
/// @author Matthew Backman
enum class ESubsystemState {
  DRIVETRAIN_GET_VELOCITY,
  DRIVETRAIN_GET_RADIUS,
  ODOMETRY_GET_POSITION,
  ODOMETRY_GET_RESETTER_RAW_VALUE
};
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif