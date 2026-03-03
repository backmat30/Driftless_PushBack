#ifndef __I_INTAKE_HPP__
#define __I_INTAKE_HPP__

#include "driftless/alliance/EAlliance.hpp"

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

  /// @brief Runs the intake to intake from the front
  /// @param voltage __double__ The voltage to supply to the intake motors
  virtual void intakeFront(double voltage) = 0;

  /// @brief Runs the intake to intake from the back
  virtual void intakeBack() = 0;

  /// @brief Intakes from the back straight to the hood
  virtual void intakeBackToHood() = 0;

  /// @brief Stops all intake motors
  virtual void stopIntake() = 0;

  /// @brief Deploys the intake "arms"
  virtual void deploy() = 0;

  /// @brief Retracts the intake "arms"
  virtual void retract() = 0;

  virtual void startColorSort(alliance::EAlliance alliance) = 0;

  /// @brief Pauses the color sorting
  virtual void pauseColorSort() = 0;

  /// @brief Resumes the color sorting
  virtual void resumeColorSort() = 0;

  /// @brief Checks if the intake arms are deployed
  /// @return __bool__ True of the arms are deployed, false otherwise
  virtual bool isDeployed() = 0;
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif