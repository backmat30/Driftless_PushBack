#ifndef __PID_HOLONOMIC_TURN_HPP__
#define __PID_HOLONOMIC_TURN_HPP__

#include <cmath>
#include <memory>

#include "driftless/control/PID.hpp"
#include "driftless/control/motion/ITurn.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"
#include "driftless/robot/subsystems/odometry/Position.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"
#include "driftless/utils/UtilityFunctions.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for control algorithms
/// @author Matthew Backman
namespace control {

/// @brief Namespace for direct motion control
/// @author Matthew Backman
namespace motion {

/// @brief Class representing a turn algorithm using a PID controller
/// @author Matthew Backman
class PIDHolonomicTurn : public ITurn {};
}  // namespace motion
}  // namespace control
}  // namespace driftless

#endif