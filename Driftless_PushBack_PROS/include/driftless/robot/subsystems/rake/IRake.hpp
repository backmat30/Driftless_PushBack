#ifndef __I_RAKE_HPP__
#define __I_RAKE_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief Namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for the rake subsystem
/// @author Matthew Backman
namespace rake {

/// @brief Generic rake object
/// @author Matthew Backman
class IRake {
 public:
  /// @brief Destroys the rake
  virtual ~IRake() = default;

  /// @brief Initializes the rake
  virtual void init() = 0;

  /// @brief Runs the rake
  virtual void run() = 0;

  /// @brief Deploys the rake
  virtual void deploy() = 0;

  /// @brief Retracts the rake
  virtual void retract() = 0;

  /// @brief Determines if the rake is deployed
  /// @return __bool__ True if deployed, otherwise false
  virtual bool isDeployed() = 0;
};
}  // namespace rake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif