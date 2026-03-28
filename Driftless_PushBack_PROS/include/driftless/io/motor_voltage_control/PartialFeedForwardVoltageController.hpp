#ifndef __PARTIAL_FEED_FORWARD_VOLTAGE_CONTROLLER_HPP__
#define __PARTIAL_FEED_FORWARD_VOLTAGE_CONTROLLER_HPP__

#include "driftless/io/motor_voltage_control/IVoltageController.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for IO device interfaces
/// @author Matthew Backman
namespace io {

/// @brief Namespace for various voltage controllers for motors
/// @author Matthew Backman
namespace motor_voltage_control {

/// @brief Interface for a voltage controller for motors
/// @author Matthew Backman
class PartialFeedForwardVoltageController : public IVoltageController {
 private:
  double m_kS{};

  double m_kV{};

 public:
  /// @brief Deletes the voltage controller
  ~PartialFeedForwardVoltageController() = default;

  /// @brief Creates a new partial feed forward voltage controller with the
  /// given constants
  /// @param kS __double__ The kS constant for the controller, in volts
  /// @param kV __double__ The kV constant for the controller, in volts per
  /// (meter per second)
  PartialFeedForwardVoltageController(double kS, double kV);

  /// @brief Gets the voltage to set the motor to
  /// @return __double__ The voltage to set the motor to, from -12.0 to 12.0
  double getVoltage(double velocity, double acceleration) override;
};
}  // namespace motor_voltage_control
}  // namespace io
}  // namespace driftless

#endif