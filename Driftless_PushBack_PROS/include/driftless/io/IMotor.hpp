#ifndef __I_MOTOR_HPP__
#define __I_MOTOR_HPP__

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for all IO device interfaces
/// @author Matthew Backman
namespace io {

/// @brief Interface for a generic motor
/// @author Matthew Backman
class IMotor {
 public:
  /// @brief Destroys the motor object
  virtual ~IMotor() = default;

  /// @brief Initialize the motor
  virtual void initialize() = 0;

  /// @brief Gets the current gear ratio of the motor
  /// @return __double__ the gear ratio
  virtual double getGearRatio() = 0;

  /// @brief Gets the angular velocity of the motor in rad/s
  /// @return __double__ the angular velocity
  virtual double getAngularVelocity() = 0;

  /// @brief Gets the position of the motor in radians
  /// @return __double__ the position of the motor
  virtual double getPosition() = 0;

  /// @brief Gets the efficiency of the motor as a percentage
  /// @return __double__ The efficiency of the motor
  virtual double getEfficiency() = 0;

  /// @brief Sets the voltage going to the motor in volts
  /// @param volts __double__ The desired voltage
  virtual void setVoltage(double volts) = 0;

  /// @brief Sets the velocity of the motor in rad/s
  /// @param velocity __double__ The desired velocity
  virtual void setVelocity(double velocity) = 0;

  /// @brief Sets the max current of the motor in amps
  /// @param amps __double__ The desired current limit
  virtual void setCurrentLimit(double amps) = 0;

  /// @brief Sets the motor encoder to be a new position
  /// @param position __double__ the desired position
  virtual void setPosition(double position) = 0;
};
}  // namespace io
}  // namespace driftless
#endif