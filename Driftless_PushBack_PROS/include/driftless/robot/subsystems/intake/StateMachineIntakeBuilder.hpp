#ifndef __STATE_MACHINE_INTAKE_BUILDER_HPP__
#define __STATE_MACHINE_INTAKE_BUILDER_HPP__

#include <memory>

#include "driftless/robot/subsystems/intake/StateMachineIntake.hpp"

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

/// @brief Builder for StateMachineIntake objects
/// @author Matthew Backman
class StateMachineIntakeBuilder {
 private:
  /// @brief The front motors used to build the StateMachineIntake
  hal::MotorGroup m_front_motors{};

  /// @brief The intermediary motors used to build the StateMachineIntake
  hal::MotorGroup m_intermediary_motors{};

  /// @brief The back motors used to build the StateMachineIntake
  hal::MotorGroup m_back_motors{};

  /// @brief The vertical transition motors used to build the StateMachineIntake
  hal::MotorGroup m_vertical_motors{};

  /// @brief The pistons used to build the StateMachineIntake
  hal::PistonGroup m_back_pistons{};

  std::unique_ptr<io::IColorSensor> m_front_color_sensor{};

  std::unique_ptr<io::IColorSensor> m_mid_color_sensor{};

  std::unique_ptr<io::IColorSensor> m_back_color_sensor{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<rtos::IClock> m_clock{};

  double m_front_sensor_distance{};

  double m_mid_sensor_distance{};

  double m_back_sensor_distance{};

  std::array<std::unique_ptr<intake_states::IIntakeState>, 16> m_states{};

 public:
  /// @brief Add a front motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor to add
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withFrontMotor(std::unique_ptr<io::IMotor>& motor);

  /// @brief Add an intermediary motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor to add
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withIntermediaryMotor(
      std::unique_ptr<io::IMotor>& motor);

  /// @brief Add a back motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor to add
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withBackMotor(std::unique_ptr<io::IMotor>& motor);

  /// @brief Add a vertical motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor to add
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withVerticalMotor(
      std::unique_ptr<io::IMotor>& motor);

  /// @brief Add a back piston to the builder
  /// @param piston __std::unique_ptr<io::IPiston>&__ The piston to add
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withBackPiston(
      std::unique_ptr<io::IPiston>& piston);

  /// @brief Add a color sensor to the builder
  /// @param piston __std::unique_ptr<io::IColorSensor>&__ The color sensor to
  /// add
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withFrontColorSensor(
      std::unique_ptr<io::IColorSensor>& color_sensor);

  /// @brief Add a color sensor to the builder
  /// @param piston __std::unique_ptr<io::IColorSensor>&__ The color sensor to
  /// add
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withMidColorSensor(
      std::unique_ptr<io::IColorSensor>& color_sensor);

  /// @brief Add a color sensor to the builder
  /// @param piston __std::unique_ptr<io::IColorSensor>&__ The color sensor to
  /// add
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withBackColorSensor(
      std::unique_ptr<io::IColorSensor>& color_sensor);

  /// @brief Add a delayer to the builder
  /// @param piston __const std::unique_ptr<rtos::IDelayer>&__ The delayer to
  /// add
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withDelayer(
      const std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Add a mutex to the builder
  /// @param piston __std::unique_ptr<rtos::IMutex>&__ The mutex to add
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Add a task to the builder
  /// @param piston __std::unique_ptr<rtos::ITask>&__ The task to add
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Add a state to the builder
  /// @param state_name __EIntakeStates__ The name of the state to add
  /// @param state __std::unique_ptr<IIntakeState>&__ The state to add
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withState(
      intake_states::EIntakeStates state_name,
      std::unique_ptr<intake_states::IIntakeState>& state);

  /// @brief Add a color sensor distance to end to the builder
  /// @param distance __double__ The distance from the color sensor to the back
  /// of the intake in inches
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withFrontColorSensorDistance(double distance);

  /// @brief Add a color sensor distance to end to the builder
  /// @param distance __double__ The distance from the color sensor to the back
  /// of the intake in inches
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withMidColorSensorDistance(double distance);

  /// @brief Add a color sensor distance to end to the builder
  /// @param distance __double__ The distance from the color sensor to the back
  /// of the intake in inches
  /// @return __StateMachineIntakeBuilder*__ Pointer to this builder
  StateMachineIntakeBuilder* withBackColorSensorDistance(double distance);

  /// @brief Builds the StateMachineIntake object
  /// @return __std::unique_ptr<IIntake>__ Pointer to the newly built
  /// StateMachineIntake
  std::unique_ptr<IIntake> build();
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif