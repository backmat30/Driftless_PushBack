#ifndef __I_BRAKE_HPP__
#define __I_BRAKE_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief Namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for the brake subsystem
/// @author Matthew Backman
namespace brake {

/// @brief Generic brake object
/// @author Matthew Backman
class IBrake {
 public:
  /// @brief Destroys the brake
  virtual ~IBrake() = default;

  /// @brief Initializes the brake
  virtual void init() = 0;

  /// @brief Runs the brake
  virtual void run() = 0;

  /// @brief Deploys the brake
  virtual void deploy() = 0;

  /// @brief Retracts the brake
  virtual void retract() = 0;

  /// @brief Determines if the brake is deployed
  /// @return __bool__ True if deployed, otherwise false
  virtual bool isDeployed() = 0;
};
}  // namespace brake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif