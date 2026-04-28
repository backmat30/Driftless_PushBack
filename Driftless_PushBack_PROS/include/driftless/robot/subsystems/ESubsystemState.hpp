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
  HOLONOMIC_DRIVE_TRAIN_GET_WHEEL_SPEED,
  ODOMETRY_GET_POSITION,
  ODOMETRY_GET_RESETTER_RAW_VALUE,
  INTAKE_ARE_ARMS_DEPLOYED,
  HOOD_IS_RAISED,
  HOOD_IS_BUMPED,
  HOOD_IS_OPEN,
  HOOD_GET_DESCORE_HEIGHT,
  BRAKE_IS_DEPLOYED,
  RAKE_IS_DEPLOYED
};
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif