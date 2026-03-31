#ifndef __A_AUTON_HPP__
#define __A_AUTON_HPP__

#include <string>

#include "driftless/alliance/EAlliance.hpp"
#include "driftless/alliance/IAlliance.hpp"
#include "driftless/control/ControlSystem.hpp"
#include "driftless/control/EControl.hpp"
#include "driftless/control/EControlCommand.hpp"
#include "driftless/control/EControlState.hpp"
#include "driftless/control/motion/ETurnDirection.hpp"
#include "driftless/control/trajectory/QuinticBezierSplinePath.hpp"
#include "driftless/control/trajectory/trajectory_generator/TrajectoryGenerator.hpp"
#include "driftless/control/trajectory/trajectory_generator/TrajectoryProcessor.hpp"
#include "driftless/control/trajectory/trajectory_generator/kinematics/CentripetalAccelerationConstraintBuilder.hpp"
#include "driftless/control/trajectory/trajectory_generator/kinematics/LinearAccelerationConstraintBuilder.hpp"
#include "driftless/control/trajectory/trajectory_generator/kinematics/MaxVelocityConstraintBuilder.hpp"
#include "driftless/control/trajectory/trajectory_generator/modifiers/HolonomicSpinModifierBuilder.hpp"
#include "driftless/processes/EProcess.hpp"
#include "driftless/processes/EProcessCommand.hpp"
#include "driftless/processes/EProcessState.hpp"
#include "driftless/processes/ProcessSystem.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/HolonomicMotionVector.hpp"
#include "driftless/robot/subsystems/odometry/Position.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/utils/UtilityFunctions.hpp"
#include "pros/screen.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for autonomous routines
/// @author Matthew Backman
namespace auton {

/// @brief Abstract class for a generic autonomous routine
/// @author Matthew Backman
class AAuton {
 private:
  static constexpr uint8_t LOOP_DELAY{10};

  std::string m_name{};

 protected:
  std::shared_ptr<robot::Robot> m_robot{};

  std::shared_ptr<control::ControlSystem> m_control_system{};

  std::shared_ptr<alliance::IAlliance> m_alliance{};

  std::shared_ptr<rtos::IClock> m_clock{};

  std::shared_ptr<rtos::IDelayer> m_delayer{};

  uint32_t getTime();

  void delay(uint32_t delay_time);

  void delayUntil(uint32_t time);

  robot::subsystems::odometry::Position getOdomPosition();

  void setOdomPosition(double x, double y, double theta);

  void followTrajectory(
      std::vector<control::trajectory::TrajectoryPoint>& trajectory);

  bool trajectoryTargetReached();

  void waitForTrajectory(control::trajectory::TrajectoryPoint& endpoint,
                         double tolerance, uint32_t timeout);

  void goToPoint(control::Point target_point, double target_velocity);

  bool goToPointTargetReached();

  void waitForGoToPoint(control::Point target_point, double tolerance,
                        uint32_t timeout);

  void setGoToPointVelocity(double velocity);

  void goToPose(control::Point target_point, double target_velocity,
                double target_angular_velocity,
                double linear_acceleration = __DBL_MAX__);

  bool goToPoseTargetReached();

  void waitForGoToPose(control::Point target_point, double position_tolerance,
                       uint32_t timeout);

  void setGoToPoseVelocity(double velocity);

  void setGoToPoseAngularVelocity(double angular_velocity);

  void turnToPoint(control::Point target_point, double target_velocity,
                   control::motion::ETurnDirection direction =
                       control::motion::ETurnDirection::AUTO);

  void turnToHeading(double heading, double target_velocity,
                     control::motion::ETurnDirection direction =
                         control::motion::ETurnDirection::AUTO);

  bool turnTargetReached();

  void waitForTurnToPoint(control::Point target_point, double tolerance,
                          uint32_t timeout);

  void waitForTurnToAngle(double heading, double tolerance, uint32_t timeout);

  void stopMotion();

  void intakeFront(double voltage = 12.0);

  void outtakeFront(double voltage = 12.0);

  void intakeBack();

  void intakeBackToHood();

  void intakeStop();

  void deployBackIntakeArms();

  void retractBackIntakeArms();

  void startColorSort(alliance::EAlliance alliance);

  void pauseColorSort();

  void hoodRaise();

  void hoodBumpUp();

  void hoodLower();

  void hoodOpenDoor();

  void hoodCloseDoor();

  void hoodSetVoltage(double voltage);

  void deployRake();

  void retractRake();

  void deployDescore();

  void middleDescore();

  void retractDescore();

 public:
  /// @brief Constructs a new AAuton object
  /// @param name __std::string__ The name of the auton
  AAuton(std::string name);

  /// @brief Deletes the auton
  virtual ~AAuton() = default;

  /// @brief Gets the name of the auton
  /// @return __std::string__ The name of the auton
  std::string getName();

  /// @brief Initializes the auton
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot being controlled
  /// @param control_system __std::shared_ptr<control::ControlSystem>&__ The
  /// control system used
  /// @param process_system __std::shared_ptr<processes::ProcessSystem>&__ The
  /// process system used
  virtual void init(
      std::shared_ptr<driftless::robot::Robot>& robot,
      std::shared_ptr<driftless::control::ControlSystem>& control_system) = 0;

  /// @brief Runs the auton
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot being controlled
  /// @param control_system __std::shared_ptr<control::ControlSystem>&__ The
  /// control system used
  /// @param process_system __std::shared_ptr<processes::ProcessSystem>&__ The
  /// process system used
  /// @param alliance __std::shared_ptr<alliance::IAlliance>&__ The current
  /// alliance
  /// @param clock __std::shared_ptr<rtos::IClock>&__ The system clock
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer used
  virtual void run(
      std::shared_ptr<driftless::robot::Robot>& robot,
      std::shared_ptr<driftless::control::ControlSystem>& control_system,
      std::shared_ptr<driftless::alliance::IAlliance>& alliance,
      std::shared_ptr<rtos::IClock>& clock,
      std::unique_ptr<rtos::IDelayer>& delayer) = 0;
};
}  // namespace auton
}  // namespace driftless
#endif