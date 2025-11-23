#ifndef __E_SUBSYSTEM_HPP__
#define __E_SUBSYSTEM_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The enum class for subsystems
/// @author Matthew Backman
enum class ESubsystem { HOLONOMIC_DRIVE_TRAIN, TANK_DRIVE_TRAIN, ODOMETRY, INTAKE, HOOD, BRAKE, RAKE };
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif