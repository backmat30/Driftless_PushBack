#include "driftless/auton/BlueSkillsAuton.hpp"

namespace driftless::auton {
BlueSkillsAuton::BlueSkillsAuton() : AAuton("Blue_Skills") {}

void BlueSkillsAuton::init(
    std::shared_ptr<robot::Robot>& robot,
    std::shared_ptr<control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;

  // control::trajectory::trajectory_generator::TrajectoryGenerator
  //     trajectory_generator{0.05};
  // control::trajectory::trajectory_generator::TrajectoryProcessor
  //     trajectory_processor{};

  // std::unique_ptr<control::trajectory::IPath> middle_blocks_path{
  //     std::make_unique<control::trajectory::QuinticBezierSplinePath>(
  //         middle_blocks_control_points)};

  // control::trajectory::trajectory_generator::kinematics::
  //     MaxVelocityConstraintBuilder lin_vel_builder{};
  // std::unique_ptr<control::trajectory::trajectory_generator::kinematics::
  //                     IKinematicConstraint>
  //     lin_vel_constraint{lin_vel_builder.withMaxVelocity(80.0)->build()};

  // control::trajectory::trajectory_generator::kinematics::
  //     LinearAccelerationConstraintBuilder lin_accel_builder{};
  // std::unique_ptr<control::trajectory::trajectory_generator::kinematics::
  //                     IKinematicConstraint>
  //     lin_accel_constraint{
  //         lin_accel_builder.withMaxAcceleration(100.0)->build()};

  // control::trajectory::trajectory_generator::kinematics::
  //     CentripetalAccelerationConstraintBuilder centr_accel_builder{};
  // std::unique_ptr<control::trajectory::trajectory_generator::kinematics::
  //                     IKinematicConstraint>
  //     centr_accel_constraint{
  //         centr_accel_builder.withMaxCentripetalAcceleration(80.0)->build()};

  // trajectory_generator.addConstraint(lin_vel_constraint);
  // trajectory_generator.addConstraint(lin_accel_constraint);
  // trajectory_generator.addConstraint(centr_accel_constraint);

  // trajectory_generator.generateTrajectory(middle_blocks_path);
  // middle_blocks_trajectory = trajectory_generator.getTrajectory();

  // control::trajectory::
}

void BlueSkillsAuton::run(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system,
    std::shared_ptr<driftless::alliance::IAlliance>& alliance,
    std::shared_ptr<rtos::IClock>& clock,
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_robot = robot;
  m_control_system = control_system;
  m_alliance = alliance;
  m_clock = clock;
  m_delayer = delayer->clone();

  // start auton
  uint32_t start_time{getTime()};
  setOdomPosition(68.25, 25.0, 0);

  // intake orange preload
  intakeFront();
  goToPoint(orange_preload, MAX_VELOCITY / 1.5);
  waitForGoToPoint(orange_preload, 2.0, 2000);
  stopMotion();

  delay(500);

  // go to the matchloader on the left of red side
  goToPoint(first_matchload_lineup, MAX_VELOCITY);
  waitForGoToPoint(first_matchload_lineup, 12.0, 4000);
  intakeStop();
  setGoToPointVelocity(MAX_VELOCITY / 2.0);
  delay(150);
  goToPoint(first_matchload, MAX_VELOCITY / 2.0);
  intakeBack();
  waitForGoToPoint(first_matchload, 2.0, 1100);
  deployBackIntakeArms();
  stopMotion();

  delay(2000);
  hoodRaise();
  delay(500);
  intakeFront();

  // score all blocks in robot
  goToPoint(left_goal_front, MAX_VELOCITY / 3.0);
  delay(500);
  retractBackIntakeArms();
  waitForGoToPoint(left_goal_front, 2.0, 2000);
  stopMotion();
  delay(250);
  hoodOpenDoor();
  intakeFront();

  delay(2000);

  goToPoint(first_matchload, MAX_VELOCITY / 2.0);
  delay(500);
  hoodCloseDoor();
  goToPoint(left_goal_front, MAX_VELOCITY / 2.5);
  waitForGoToPoint(left_goal_front, 1.0, 1250);

  // grab 2 blocks on the left wall
  goToPoint(front_left_wall_blocks_lineup, MAX_VELOCITY);
  intakeFront();
  waitForGoToPoint(front_left_wall_blocks_lineup, 6.0, 1000);
  hoodLower();
  goToPoint(front_left_wall_blocks, MAX_VELOCITY / 2.0);
  waitForGoToPoint(front_left_wall_blocks, 3.0, 1000);
  stopMotion();
  delay(250);
  deployRake();
  goToPoint(front_left_wall_blocks_lineup, MAX_VELOCITY / 2.0);
  waitForGoToPoint(front_left_wall_blocks_lineup, 6.0, 1000);

  // grab park zone blocks
  goToPoint(park_zone_blocks_lineup, MAX_VELOCITY);
  waitForGoToPoint(park_zone_blocks_lineup, 12.0, 3000);
  setGoToPointVelocity(MAX_VELOCITY / 3.0);
  waitForGoToPoint(park_zone_blocks_lineup, 2.0, 1000);
  goToPoint(park_zone_blocks, MAX_VELOCITY);
  retractRake();
  waitForGoToPoint(park_zone_blocks, 14.0, 1000);
  setGoToPointVelocity(MAX_VELOCITY / 4.0);
  waitForGoToPoint(park_zone_blocks, 2.0, 1500);
  deployRake();
  delay(500);
  goToPoint(park_zone_blocks_lineup, MAX_VELOCITY / 3.0);
  waitForGoToPoint(park_zone_blocks_lineup, 2.0, 1000);

  delay(150);

  // grab block 7 for the top middle goal
  turnToHeading(middle_blocks_lineup.getTheta(), M_PI * 1.0);
  delay(500);
  goToPoint(middle_blocks_lineup, MAX_VELOCITY);
  waitForGoToPoint(middle_blocks_lineup, 8.0, 4000);
  goToPoint(middle_blocks_lineup_2, MAX_VELOCITY / 3.0);
  waitForGoToPoint(middle_blocks_lineup_2, 12.0, 1000);
  setGoToPointVelocity(MAX_VELOCITY / 3.0);
  waitForGoToPoint(middle_blocks_lineup_2, 2.0, 4000);
  stopMotion();
  turnToHeading(middle_blocks_end.getTheta(), M_PI * 3.0);
  delay(500);
  retractRake();
  goToPoint(middle_blocks_end, MAX_VELOCITY / 2.0);
  waitForGoToPoint(middle_blocks_end, 2.0, 2000);
  stopMotion();

  delay(500);

  // block lower middle goal for other bot
  goToPoint(middle_top_goal_lineup, MAX_VELOCITY);
  waitForGoToPoint(middle_top_goal_lineup, 2.0, 1000);
  intakeStop();
  goToPoint(middle_lower_goal_lineup, MAX_VELOCITY);
  waitForGoToPoint(middle_lower_goal_lineup, 2.0, 2000);
  goToPoint(middle_lower_goal, MAX_VELOCITY / 3.0);
  waitForGoToPoint(middle_lower_goal, 2.0, 2000);
  stopMotion();

  delayUntil(start_time + 32500);

  goToPoint(middle_lower_goal_lineup, MAX_VELOCITY);
  waitForGoToPoint(middle_lower_goal_lineup, 4.0, 2000);

  // score on middle goal top
  startColorSort(alliance::EAlliance::BLUE);
  goToPoint(middle_top_goal_lineup, MAX_VELOCITY);
  waitForGoToPoint(middle_top_goal_lineup, 2.0, 4000);
  hoodBumpUp();
  goToPoint(middle_top_goal, MAX_VELOCITY);
  delay(250);
  setGoToPointVelocity(MAX_VELOCITY / 3.0);
  waitForGoToPoint(middle_top_goal, 2.0, 3000);
  stopMotion();
  hoodOpenDoor();
  intakeFront();
  hoodSetVoltage(6.0);

  delay(1200);
  hoodSetVoltage(9.0);
  delay(1200);
  hoodRaise();
  pauseColorSort();

  goToPoint(middle_top_goal_lineup, MAX_VELOCITY);
  waitForGoToPoint(middle_top_goal_lineup, 2.0, 1000);
  intakeStop();
  hoodLower();
  hoodCloseDoor();

  // left wall blocks on the far side
  startColorSort(alliance::EAlliance::RED);
  intakeFront();
  goToPoint(back_left_wall_blocks_lineup_1, MAX_VELOCITY);
  waitForGoToPoint(back_left_wall_blocks_lineup_1, 16.0, 5000);
  goToPoint(back_left_wall_blocks_lineup_2, MAX_VELOCITY);
  waitForGoToPoint(back_left_wall_blocks_lineup_2, 2.0, 2000);
  goToPoint(back_left_wall_blocks, MAX_VELOCITY / 1.5);
  waitForGoToPoint(back_left_wall_blocks, 16.0, 2000);
  setGoToPointVelocity(MAX_VELOCITY / 3.0);
  waitForGoToPoint(back_left_wall_blocks, 2.0, 1500);
  stopMotion();
  deployRake();
  delay(250);
  goToPoint(back_left_wall_blocks_lineup_2, MAX_VELOCITY / 2.0);
  delay(500);
  retractRake();
  goToPoint(back_left_wall_blocks, MAX_VELOCITY / 2.0);
  waitForGoToPoint(back_left_wall_blocks, 5.0, 1000);
  delay(100);
  pauseColorSort();

  // back matchloader
  goToPoint(second_matchload_lineup, MAX_VELOCITY);
  waitForGoToPoint(second_matchload_lineup, 12.0, 1500);
  setGoToPointVelocity(MAX_VELOCITY / 2.0);
  waitForGoToPoint(second_matchload_lineup, 2.0, 2500);
  goToPoint(second_matchload, MAX_VELOCITY);
  waitForGoToPoint(second_matchload, 6.0, 1500);
  setGoToPointVelocity(MAX_VELOCITY / 3.0);
  waitForGoToPoint(second_matchload, 2.0, 750);
  deployBackIntakeArms();
  intakeBack();

  delay(2000);
  hoodRaise();
  delay(250);
  intakeFront();

  // score on long goal
  goToPoint(left_goal_back, MAX_VELOCITY);
  waitForGoToPoint(left_goal_back, 12.0, 2000);
  retractBackIntakeArms();
  setGoToPointVelocity(MAX_VELOCITY / 3.0);
  waitForGoToPoint(left_goal_back, 2.0, 1250);
  hoodOpenDoor();
  intakeFront();

  delay(2250);

  // go back to match loader quick in case we didnt get all
  goToPoint(second_matchload, MAX_VELOCITY / 1.5);
  waitForGoToPoint(second_matchload, 2.0, 1250);
  deployBackIntakeArms();
  hoodLower();
  hoodCloseDoor();
  intakeBack();
  delay(250);

  // go to the park zone
  retractBackIntakeArms();
  intakeStop();
  goToPoint(park_lineup_1, MAX_VELOCITY);
  waitForGoToPoint(park_lineup_1, 10.0, 2000);
  goToPoint(park_lineup_2, MAX_VELOCITY);
  waitForGoToPoint(park_lineup_2, 10.0, 2000);
  goToPoint(park_zone, MAX_VELOCITY / 1.5);

  delayUntil(start_time + 60000);

  // leave at end
  stopMotion();
  intakeFront();
  hoodBumpUp();
  hoodOpenDoor();
  retractBackIntakeArms();
  retractRake();
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 8, "Runtime: %7.2f",
                      (getTime() - start_time) / 1000.0);
}
}  // namespace driftless::auton