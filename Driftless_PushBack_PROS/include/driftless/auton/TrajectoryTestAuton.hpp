#ifndef __TRAJECTORY_TEST_AUTON_HPP__
#define __TRAJECTORY_TEST_AUTON_HPP__

#include "driftless/auton/IAuton.hpp"
#include "driftless/control/trajectory/trajectory_generator/TrajectoryGenerator.hpp"
#include "driftless/control/trajectory/trajectory_generator/TrajectoryProcessor.hpp"
#include "driftless/control/trajectory/trajectory_generator/kinematics/LinearAccelerationConstraintBuilder.hpp"
#include "driftless/control/trajectory/trajectory_generator/kinematics/MaxVelocityConstraintBuilder.hpp"
#include "driftless/control/trajectory/trajectory_generator/kinematics/CentripetalAccelerationConstraintBuilder.hpp"
#include "driftless/control/trajectory/QuinticBezierSplinePath.hpp"
#include "driftless/robot/subsystems/odometry/Position.hpp"
#include "driftless/utils/UtilityFunctions.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/HolonomicMotionVector.hpp"
#include "driftless/control/trajectory/trajectory_generator/modifiers/HolonomicSpinModifierBuilder.hpp"

namespace driftless {
namespace auton {
class TrajectoryTestAuton : public IAuton {
 private:
  static constexpr uint8_t LOOP_DELAY{10};

  static constexpr char AUTON_NAME[]{"TRAJECTORY TEST"};

  std::shared_ptr<rtos::IClock> m_clock{};

  std::unique_ptr<rtos::IDelayer> m_delayer{};

  std::shared_ptr<robot::Robot> m_robot{};

  std::shared_ptr<control::ControlSystem> m_control_system{};

  std::shared_ptr<alliance::IAlliance> m_alliance{};

  std::vector<control::trajectory::TrajectoryPoint> m_trajectory{};

  uint32_t getTime();

  robot::subsystems::odometry::Position getOdomPosition();

  void setOdomPosition(double x, double y, double theta);

  void followTrajectory(
      std::vector<control::trajectory::TrajectoryPoint>& trajectory);

  bool trajectoryTargetReached();

  void waitForTrajectory(control::trajectory::TrajectoryPoint& endpoint,
                         double tolerance, uint32_t timeout);

  void stopMotion();

 public:
  std::string getName() override;

  void init(std::shared_ptr<robot::Robot>& robot,
            std::shared_ptr<control::ControlSystem>& control_system) override;

  void run(std::shared_ptr<robot::Robot>& robot,
           std::shared_ptr<control::ControlSystem>& control_system,
           std::shared_ptr<alliance::IAlliance>& alliance,
           std::shared_ptr<rtos::IClock>& clock,
           std::unique_ptr<rtos::IDelayer>& delayer) override;
};
}  // namespace auton
}  // namespace driftless
#endif