#ifndef __STATE_MACHINE_INTAKE_HPP__
#define __STATE_MACHINE_INTAKE_HPP__

#include <array>

#include "driftless/hal/MotorGroup.hpp"
#include "driftless/hal/PistonGroup.hpp"
#include "driftless/io/IColorSensor.hpp"
#include "driftless/robot/subsystems/intake/IIntake.hpp"
#include "driftless/robot/subsystems/intake/intake_states/EIntakeStates.hpp"
#include "driftless/robot/subsystems/intake/intake_states/IIntakeState.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"

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

/// @brief Intake using a state machine to handle behaviour
/// @author Matthew Backman
class StateMachineIntake : public IIntake {
 private:
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Constantly runs task updates
  /// @param params __void*__ Pointer to the DirectIntake object being updated
  static void taskLoop(void* params);

  /// @brief The delayer for the intake
  std::unique_ptr<rtos::IDelayer> m_delayer{};

  /// @brief The mutex for the intake
  std::unique_ptr<rtos::IMutex> m_mutex{};

  /// @brief The task for the intake
  std::unique_ptr<rtos::ITask> m_task{};

  /// @brief The motors used by the front intake
  hal::MotorGroup m_front_motors{};

  /// @brief The motors used by the intermediary section of the intake
  hal::MotorGroup m_intermediary_motors{};

  /// @brief The motors used for the back intake
  hal::MotorGroup m_back_motors{};

  /// @brief The motors used for the vertical transition to the hood
  hal::MotorGroup m_vertical_motors{};

  /// @brief The pistons used by the back intake
  hal::PistonGroup m_back_pistons{};

  /// @brief The color sensor used for color sorting
  std::unique_ptr<io::IColorSensor> m_color_sensor{};

  alliance::EAlliance m_alliance{alliance::EAlliance::NONE};

  intake_states::EIntakeStates current_state{
      intake_states::EIntakeStates::IDLE};

  std::array<std::unique_ptr<intake_states::IIntakeState>, 16> m_states{};

  double m_color_sensor_distance_to_end{1.5};

  double m_desired_voltage{};

  bool m_color_sort_paused{true};

  /// @brief Performs all instance related updates
  void taskUpdate();

 public:
  /// @brief Initializes the intake
  void init() override;

  /// @brief Runs the intake
  void run() override;

  /// @brief Runs the intake to intake from the front
  /// @param voltage __double__ The voltage to supply to the intake motors
  void intakeFront(double voltage) override;

  /// @brief Runs the intake to intake from the back
  void intakeBack() override;

  /// @brief Intakes from the back straight to the hood
  void intakeBackToHood() override;

  /// @brief Stops all intake motors
  void stopIntake() override;

  /// @brief Starts the color sorting process
  /// @param alliance __alliance::EAlliance__ The alliance color to sort for
  void startColorSort(alliance::EAlliance alliance) override;

  /// @brief Pauses the color sorting
  void pauseColorSort() override;

  /// @brief Resumes the color sorting
  void resumeColorSort() override;

  /// @brief Deploys the intake "arms"
  void deploy() override;

  /// @brief Retracts the intake "arms"
  void retract() override;

  /// @brief Checks if the intake arms are deployed
  /// @return __bool__ True of the arms are deployed, false otherwise
  bool isDeployed() override;

  /// @brief Sets the voltage for the front intake motors
  /// @param voltage __double__ The new voltage
  void setFrontMotorVoltage(double voltage);

  /// @brief Sets the current limit for the front intake motors
  /// @param amps __double__ The new current limit in amps
  void setFrontMotorCurrentLimit(double amps);

  /// @brief Sets the voltage for the intermediary intake motors
  /// @param voltage __double__ The new voltage
  void setIntermediaryMotorVoltage(double voltage);

  /// @brief Sets the current limit for the intermediary intake motors
  /// @param amps __double__ The new current limit in amps
  void setIntermediaryMotorCurrentLimit(double amps);

  /// @brief Sets the voltage for the back intake motors
  /// @param voltage __double__ The new voltage
  void setBackMotorVoltage(double voltage);

  /// @brief Sets the current limit for the back intake motors
  /// @param amps __double__ The new current limit in amps
  void setBackMotorCurrentLimit(double amps);

  /// @brief Sets the voltage for the vertical motors
  /// @param voltage __double__ The new voltage
  void setVerticalMotorVoltage(double voltage);

  /// @brief Sets the current limit for the vertical motors
  /// @param amps __double__ The new current limit in amps
  void setVerticalMotorCurrentLimit(double amps);

  /// @brief Sets the current state of the intake
  /// @param state The new state
  void setCurrentState(intake_states::EIntakeStates state);

  /// @brief Checks if there is a block of any kind in the intake
  /// @return __bool__ True if there is a block, false otherwise
  bool hasBlock();

  /// @brief Checks if there is a block of the opposing alliance in the intake
  /// @return __bool__ True if there is an opposing block, false otherwise
  bool hasOpposingBlock();

  /// @brief Checks if there is a block of the current alliance in the intake
  /// @return __bool__ True if there is an alliance block, false otherwise
  bool hasAllianceBlock();

  /// @brief Gets the position of the front intake motors
  /// @return __double__ The position of the front intake motors
  double getFrontMotorPosition();

  /// @brief Gets the position of the color sensor relative to the back intake
  /// @return __double__ The position of the color sensor relative to the back
  /// intake
  double getColorSensorPosition();

  /// @brief Checks if the color sorting is paused
  /// @return __bool__ True if the color sorting is paused, false otherwise
  bool isColorSortPaused();

  /// @brief Gets the desired voltage for the intake motors
  /// @return __double__ The desired voltage for the intake motors
  double getDesiredVoltage();

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

  /// @brief Sets the motors used for the vertical transition to the hood
  /// @param motors __hal::MotorGroup&__ The motors to use
  void setVerticalMotors(hal::MotorGroup& motors);

  /// @brief Sets the distance from the color sensor to the back of the intake
  /// @param distance __double__ The distance from the color sensor to the back
  /// of the intake in inches
  void setColorSensorDistanceToEnd(double distance);

  /// @brief Sets the color sensor used for color sorting
  /// @param color_sensor __std::unique_ptr<io::IColorSensor>&__ The color
  /// sensor to use
  void setColorSensor(std::unique_ptr<io::IColorSensor>& color_sensor);

  /// @brief Sets the delayer used by the intake
  /// @param delayer __const std::unique_ptr<rtos::IDelayer>&__ The delayer to
  /// use
  void setDelayer(const std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Sets the mutex used by the intake
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex to use
  void setMutex(std::unique_ptr<rtos::IMutex>& mutex);

  /// @brief Sets the task used by the intake
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task to use
  void setTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Sets the states for the intake state machine
  /// @param states __array<std::unique_ptr<intake_states::IIntakeState>, 16>&__
  /// The states to use
  void setStates(
      std::array<std::unique_ptr<intake_states::IIntakeState>, 16>& states);
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif