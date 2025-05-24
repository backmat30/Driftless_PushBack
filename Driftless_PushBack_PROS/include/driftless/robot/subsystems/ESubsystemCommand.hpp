#ifndef __E_SUBSYSTEM_COMMAND_HPP__
#define __E_SUBSYSTEM_COMMAND_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {
  
/// @brief The enum class for subsystem commands
/// @author Matthew Backman
enum class ESubsystemCommand {
  DRIVETRAIN_SET_VELOCITY,
  DRIVETRAIN_SET_VOLTAGE,
  ODOMETRY_SET_POSITION,
  ODOMETRY_SET_X,
  ODOMETRY_SET_Y,
  ODOMETRY_SET_THETA,
  ODOMETRY_RESET_X,
  ODOMETRY_RESET_Y

};
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif