#ifndef __DIRECT_HOOD_HPP__
#define __DIRECT_HOOD_HPP__

#include <memory>

#include "driftless/hal/MotorGroup.hpp"
#include "driftless/hal/PistonGroup.hpp"
#include "driftless/robot/subsystems/hood/IHood.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for the hood subsystem
/// @author Matthew Backman
namespace hood {

/// @brief Directly driven hood implementation
/// @author Matthew Backman
class DirectHood : public IHood {
 private:
  /// @brief Whether the hood is raised or not
  bool m_raised{};

  /// @brief Whether the gate on the hood is open or not
  bool m_gate_open{};

  /// @brief The motors used by the hood
  hal::MotorGroup m_motors{};

  /// @brief The pistons controlling the height of the hood
  hal::PistonGroup m_height_pistons{};

  /// @brief The pistons controlling the hood gate
  hal::PistonGroup m_gate_pistons{};

  /// @brief The pistons controlling the descore mechanism
  hal::PistonGroup m_descore_pistons{};

  /// @brief The pistons controlling the hood bump
  hal::PistonGroup m_bump_pistons{};

 public:
  /// @brief Initializes the hood
  void init() override;

  /// @brief Runs the hood
  void run() override;

  /// @brief Sets the voltage of the hood motors
  /// @param voltage __double__ The voltage
  void setVoltage(double voltage) override;

  /// @brief Opens the gate at the end of the hood
  void open() override;

  /// @brief Closes the gate at the end of the hood
  void close() override;

  /// @brief Toggles the gate between open and closed
  void toggleOpen() override;

  /// @brief Raises the hood
  void raise() override;

  /// @brief Lowers the hood
  void lower() override;

  /// @brief Toggles the hood between raised and lowered
  void toggleRaised() override;

  /// @brief Extends the descore mechanism
  void extendDescore() override;

  /// @brief Retracts the descore mechanism
  void retractDescore() override;

  /// @brief Toggles the descore mechanism between extended and retracted
  void toggleDescore() override;

  /// @brief Bumps the hood up slightly
  void bumpUp() override;

  /// @brief Retracts the hood bump
  void bumpDown() override;

  /// @brief Toggles the hood bump
  void toggleBump() override;

  /// @brief Checks if the hood is raised
  /// @return __bool__ True if the hood is raised, false otherwise
  bool isRaised() override;

  /// @brief Checks if the hood bump is extended
  /// @return __bool__ True if the hood is bumped up, false otherwise
  bool isBumped() override;

  /// @brief Checks if the hood door is open
  /// @return __bool__ True if the door is open, false otherwise
  bool isOpen() override;

  /// @brief Sets the motors used by the hood
  /// @param motors __hal::MotorGroup&__ The motors to use
  void setMotors(hal::MotorGroup& motors);

  /// @brief Sets the pistons to use for height control
  /// @param pistons __hal::PistonGroup&__ The pistons to use
  void setHeightPistons(hal::PistonGroup& pistons);

  /// @brief Sets the pistons to use for gate control
  /// @param pistons __hal::PistonGroup&__ The pistons to use
  void setGatePistons(hal::PistonGroup& pistons);

  /// @brief Sets the pistons to use for descore control
  /// @param pistons __hal::PistonGroup&__ The pistons to use
  void setDescorePistons(hal::PistonGroup& pistons);

  /// @brief Sets the pistons to use for bump control
  /// @param pistons __hal::PistonGroup&__ The pistons to use
  void setBumpPistons(hal::PistonGroup& pistons);
};
}  // namespace hood
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif