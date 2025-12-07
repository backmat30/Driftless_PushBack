#ifndef __DIRECT_INTAKE_HPP__
#define __DIRECT_INTAKE_HPP__

#include "driftless/hal/MotorGroup.hpp"
#include "driftless/hal/PistonGroup.hpp"
#include "driftless/io/IColorSensor.hpp"
#include "driftless/robot/subsystems/intake/IIntake.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"
#include "driftless/rtos/IClock.hpp"

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
  static constexpr uint8_t TASK_DELAY{10};

  static constexpr double COLOR_SORT_DISTANCE_TO_END{5.5};

  /// @brief Constantly runs task updates
  /// @param params __void*__ Pointer to the DirectIntake object being updated
  static void taskLoop(void* params);

  /// @brief The delayer for the intake
  std::unique_ptr<rtos::IDelayer> m_delayer{};

  /// @brief The mutex for the intake
  std::unique_ptr<rtos::IMutex> m_mutex{};

  /// @brief The task for the intake
  std::unique_ptr<rtos::ITask> m_task{};

  std::unique_ptr<rtos::IClock> m_clock{};

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

  bool m_color_sort_paused{true};

  bool m_running_forward{};

  bool m_back_intake_to_hood{};

  double m_latest_empty_intake_time{};

  double m_latest_opposing_block_pos{-__DBL_MAX__};

  /// @brief Checks if there is a block of the opposing alliance in the intake
  /// @return __bool__ True if there is an opposing block, false otherwise
  bool hasOpposingBlock();

  /// @brief Performs all instance related updates
  void taskUpdate();

 public:
  /// @brief Initializes the intake
  void init() override;

  /// @brief Runs the intake
  void run() override;

  /// @brief Runs the intake to intake from the front
  /// @param reversed __bool__ True for outtaking, false for intaking
  void intakeFront(bool reversed) override;

  /// @brief Runs the intake to intake from the back
  void intakeBack() override;

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

  /// @brief Sets the clock used by the intake
  /// @param clock __const std::unique_ptr<rtos::IClock>&__ The clock to use
  void setClock(const std::unique_ptr<rtos::IClock>& clock);
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif