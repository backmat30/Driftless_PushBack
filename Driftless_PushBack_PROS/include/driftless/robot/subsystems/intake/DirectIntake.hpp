#ifndef __DIRECT_INTAKE_HPP__
#define __DIRECT_INTAKE_HPP__

#include "driftless/hal/MotorGroup.hpp"
#include "driftless/hal/PistonGroup.hpp"
#include "driftless/robot/subsystems/intake/IIntake.hpp"

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

/// @brief Intake directly ran by motors and pistons
/// @author Matthew Backman
class DirectIntake : public IIntake {
 private:
  /// @brief The motors used by the intake
  hal::MotorGroup m_motors{};

  /// @brief The pistons used by the intake
  hal::PistonGroup m_pistons{};

 public:
  /// @brief Initializes the intake
  void init() override;

  /// @brief Runs the intake
  void run() override;

  /// @brief Sets the voltage of the intake motors
  /// @param voltage __double__ The voltage to use
  void setVoltage(double voltage) override;

  /// @brief Deploys the intake "arms"
  void deploy() override;

  /// @brief Retracts the intake "arms"
  void retract() override;

  /// @brief Sets the motors used by the intake
  /// @param motors __hal::MotorGroup&__ The motors to use
  void setMotors(hal::MotorGroup& motors);

  /// @brief Sets the pistons used by the intake
  /// @param pistons __hal::PistonGroup&__ The pistons to use
  void setPistons(hal::PistonGroup& pistons);
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif