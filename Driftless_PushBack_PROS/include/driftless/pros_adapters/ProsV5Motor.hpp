#ifndef PROS_ADAPTERS_PROS_V5_MOTOR_HPP
#define PROS_ADAPTERS_PROS_V5_MOTOR_HPP

#include <cmath>
#include <map>
#include <memory>

#include "driftless/io/IMotor.hpp"
#include "driftless/io/motor_voltage_control/IVoltageController.hpp"
#include "pros/motors.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for PROS adapters
/// @author Matthew Backman
namespace pros_adapters {

/// @brief Class to adapt the pros motor class to the IMotor interface
/// @author Matthew Backman
class ProsV5Motor : public io::IMotor {
 private:
  // maps pros motor cartridges to the corresponding gear ratios
  const std::map<pros::MotorGears, double> cartridge_map{
      {pros::MotorGears::rpm_100, 36.0},
      {pros::MotorGears::rpm_200, 18.0},
      {pros::MotorGears::rpm_600, 6.0}};

  /**
   * @brief The gear ratio if no cartridge is present
   *
   */
  static constexpr double NO_CARTRIDGE{1.0};

  /**
   * @brief Converts motor velocity to radians/second
   *
   */
  static constexpr double VELOCITY_CONVERSION{2 * M_PI / 60};

  /**
   * @brief Converts motor position to radians
   *
   */
  static constexpr double POSITION_CONVERSION{M_PI * 2};

  /**
   * @brief Converts input voltage to millivolts
   *
   */
  static constexpr double VOLTAGE_CONVERSION{1000};

  static constexpr double CURRENT_CONVERSION{1000};

  /**
   * @brief The maximum output to the motor in millivolts
   *
   */
  static constexpr int MAX_MILLIVOLTS{12000};

  static constexpr int MAX_MILLIAMPS{2500};

  /**
   * @brief The motor being adapted
   *
   */
  std::unique_ptr<pros::Motor> m_motor{};

  /// @brief The voltage controller used to find voltage when given a velocity
  std::unique_ptr<io::motor_voltage_control::IVoltageController>
      voltage_controller{};

  /**
   * @brief The position offset
   *
   */
  double position_offset{};

 public:
  /**
   * @brief Construct a new Pros V5 Motor object
   *
   * @param motor __std::unique_ptr<pros::Motor>&__ The motor being adapted
   */
  ProsV5Motor(std::unique_ptr<pros::Motor>& motor);

  /// @brief Construct a new Pros V5 Motor object with a voltage controller
  /// @param motor __std::unique_ptr<pros::Motor>&__ The motor being adapted
  /// @param voltage_controller
  /// __std::unique_ptr<io::motor_voltage_control::IVoltageController>&__ The
  /// voltage controller to use for velocity control
  ProsV5Motor(std::unique_ptr<pros::Motor>& motor,
              std::unique_ptr<io::motor_voltage_control::IVoltageController>&
                  voltage_controller);

  /**
   * @brief Initializes the motor
   *
   */
  void initialize() override;

  /**
   * @brief Get the gear ratio of the motor (1 if n/a)
   *
   * @return __double__ The gear ratio of the motor
   */
  double getGearRatio() override;

  /**
   * @brief Get the angular velocity of the motor in radians/second
   *
   * @return __double__ The angular velocity of the motor in radians/second
   */
  double getAngularVelocity() override;

  /**
   * @brief Get the position of the motor in total radians
   *
   * @return __double__ The total number of radians moved since last reset
   */
  double getPosition() override;

  /// @brief Gets the efficiency of the motor as a percentage
  /// @return __double__ The efficiency
  double getEfficiency() override;

  /**
   * @brief Set the voltage input to the motor in Volts
   *
   * @param volts __double__ The voltage input in Volts
   */
  void setVoltage(double volts) override;

  /// @brief Sets the velocity of the motor in rad/s
  /// @param velocity __double__ The desired velocity
  void setVelocity(double velocity) override;

  /// @brief Sets the max current of the motor in amps
  /// @param amps __double__ The desired current limit
  void setCurrentLimit(double amps) override;

  /**
   * @brief Set the position of the motor in radians
   *
   * @param position __double__ The position of the motor
   */
  void setPosition(double position) override;
};
}  // namespace pros_adapters
}  // namespace driftless
#endif