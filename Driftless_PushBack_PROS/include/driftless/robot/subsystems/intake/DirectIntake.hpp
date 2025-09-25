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
  /// @brief The motors used by the front intake
  hal::MotorGroup m_front_motors{};

  /// @brief The motors used by the intermediary section of the intake
  hal::MotorGroup m_intermediary_motors{};

  /// @brief The motors used for the back intake
  hal::MotorGroup m_back_motors{};

  /// @brief The pistons used by the back intake
  hal::PistonGroup m_back_pistons{};

 public:
  /// @brief Initializes the intake
  void init() override;

  /// @brief Runs the intake
  void run() override;

  /// @brief Sets the voltage of the front intake motors
  /// @param voltage __double__ The voltage to use
  void setFrontVoltage(double voltage) override;

  /// @brief Sets the voltage of the intermediary motors
  /// @param voltage __double__ The voltage to use
  void setIntermediaryVoltage(double voltage) override;

  /// @brief Sets the voltage of the back intake motors
  /// @param voltage __double__ The voltage to use
  void setBackVoltage(double voltage) override;

  /// @brief Deploys the intake "arms"
  void deploy() override;

  /// @brief Retracts the intake "arms"
  void retract() override;

  /// @brief Sets the motors used by the front intake
  /// @param motors __hal::MotorGroup&__ The motors to use
  void setFrontMotors(hal::MotorGroup& motors);

  /// @brief Sets the intermediary motors for the intake
  /// @param motors __hal::MotorGroup&__ The motors to use
  void setIntermediaryMotors(hal::MotorGroup& motors);

  /// @brief Sets the motors for the back intake
  /// @param motors __hal::MotorGroup&__ The motors to use
  void setBackMotors(hal::MotorGroup& motors);

  /// @brief Sets the pistons used by the back intake
  /// @param pistons __hal::PistonGroup&__ The pistons to use
  void setBackPistons(hal::PistonGroup& pistons);
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif