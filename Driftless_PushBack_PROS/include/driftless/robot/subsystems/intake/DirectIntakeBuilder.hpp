#ifndef __DIRECT_INTAKE_BUILDER_HPP__
#define __DIRECT_INTAKE_BUILDER_HPP__

#include <memory>

#include "driftless/robot/subsystems/intake/DirectIntake.hpp"

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

/// @brief Builder for DirectIntake objects
/// @author Matthew Backman
class DirectIntakeBuilder {
 private:
  /// @brief The front motors used to build the DirectIntake
  hal::MotorGroup m_front_motors{};

  /// @brief The intermediary motors used to build the DirectIntake
  hal::MotorGroup m_intermediary_motors{};

  /// @brief The back motors used to build the DirectIntake
  hal::MotorGroup m_back_motors{};

  /// @brief The vertical transition motors used to build the DirectIntake
  hal::MotorGroup m_vertical_motors{};

  /// @brief The pistons used to build the DirectIntake
  hal::PistonGroup m_back_pistons{};

  std::unique_ptr<io::IColorSensor> m_color_sensor{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::unique_ptr<rtos::IMutex> m_mutex{};

  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<rtos::IClock> m_clock{};

 public:
  /// @brief Add a front motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor to add
  /// @return __DirectIntakeBuilder*__ Pointer to this builder
  DirectIntakeBuilder* withFrontMotor(std::unique_ptr<io::IMotor>& motor);

  /// @brief Add an intermediary motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor to add
  /// @return __DirectIntakeBuilder*__ Pointer to this builder
  DirectIntakeBuilder* withIntermediaryMotor(
      std::unique_ptr<io::IMotor>& motor);

  /// @brief Add a back motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor to add
  /// @return __DirectIntakeBuilder*__ Pointer to this builder
  DirectIntakeBuilder* withBackMotor(std::unique_ptr<io::IMotor>& motor);

  /// @brief Add a vertical motor to the builder
  /// @param motor __std::unique_ptr<io::IMotor>&__ The motor to add
  /// @return __DirectIntakeBuilder*__ Pointer to this builder
  DirectIntakeBuilder* withVerticalMotor(std::unique_ptr<io::IMotor>& motor);

  /// @brief Add a back piston to the builder
  /// @param piston __std::unique_ptr<io::IPiston>&__ The piston to add
  /// @return __DirectIntakeBuilder*__ Pointer to this builder
  DirectIntakeBuilder* withBackPiston(std::unique_ptr<io::IPiston>& piston);

  /// @brief Add a color sensor to the builder
  /// @param piston __std::unique_ptr<io::IColorSensor>&__ The color sensor to add
  /// @return __DirectIntakeBuilder*__ Pointer to this builder
  DirectIntakeBuilder* withColorSensor(std::unique_ptr<io::IColorSensor>& color_sensor);

  /// @brief Add a delayer to the builder
  /// @param piston __const std::unique_ptr<rtos::IDelayer>&__ The delayer to add
  /// @return __DirectIntakeBuilder*__ Pointer to this builder
  DirectIntakeBuilder* withDelayer(const std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Add a mutex to the builder
  /// @param piston __std::unique_ptr<rtos::IMutex>&__ The mutex to add
  /// @return __DirectIntakeBuilder*__ Pointer to this builder
  DirectIntakeBuilder* withMutex(std::unique_ptr<rtos::IMutex>& mutex);

    /// @brief Add a task to the builder
  /// @param piston __std::unique_ptr<rtos::ITask>&__ The task to add
  /// @return __DirectIntakeBuilder*__ Pointer to this builder
  DirectIntakeBuilder* withTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Builds the DirectIntake object
  /// @return __std::unique_ptr<IIntake>__ Pointer to the newly built
  /// DirectIntake
  std::unique_ptr<IIntake> build();
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif