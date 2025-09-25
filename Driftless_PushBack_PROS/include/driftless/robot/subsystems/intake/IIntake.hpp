#ifndef __I_INTAKE_HPP__
#define __I_INTAKE_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for the intake subsystem
/// @author Matthew Backman
namespace intake {

/// @brief Interface for a generic intake
/// @author Matthew Backman
class IIntake {
 public:
  /// @brief Destroys the intake object
  virtual ~IIntake() = default;

  /// @brief Initializes the intake
  virtual void init() = 0;

  /// @brief Runs the intake
  virtual void run() = 0;

  /// @brief Sets the voltage of the front intake motors
  /// @param voltage __double__ The voltage to use
  virtual void setFrontVoltage(double voltage) = 0;

  /// @brief Sets the voltage of the intermediary motors
  /// @param voltage __double__ The voltage to use
  virtual void setIntermediaryVoltage(double voltage) = 0;

  /// @brief Sets the voltage of the back intake motors
  /// @param voltage __double__ The voltage to use
  virtual void setBackVoltage(double voltage) = 0;

  /// @brief Deploys the intake "arms"
  virtual void deploy() = 0;

  /// @brief Retracts the intake "arms"
  virtual void retract() = 0;

  /// @brief Checks if the intake arms are deployed
  /// @return __bool__ True of the arms are deployed, false otherwise
  virtual bool isDeployed() = 0;
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif