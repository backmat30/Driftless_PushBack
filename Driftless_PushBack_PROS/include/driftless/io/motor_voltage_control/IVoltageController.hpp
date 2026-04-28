#ifndef __I_VOLTAGE_CONTROLLER_HPP__
#define __I_VOLTAGE_CONTROLLER_HPP__

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
class IVoltageController {
 public:
  /// @brief Deletes the voltage controller
  virtual ~IVoltageController() = default;

  /// @brief Gets the voltage to set the motor to
  /// @return __double__ The voltage to set the motor to, from -12.0 to 12.0
  virtual double getVoltage(double velocity, double acceleration) = 0;
};
}  // namespace motor_voltage_control
}  // namespace io
}  // namespace driftless

#endif