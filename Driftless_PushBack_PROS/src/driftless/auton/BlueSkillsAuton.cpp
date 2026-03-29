#include "driftless/auton/BlueSkillsAuton.hpp"

namespace driftless::auton {
BlueSkillsAuton::BlueSkillsAuton() : AAuton("Blue_Skills") {}

void BlueSkillsAuton::init(
    std::shared_ptr<robot::Robot>& robot,
    std::shared_ptr<control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
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
  goToPose(orange_preload, MAX_VELOCITY / 2.5, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(orange_preload, 2.0, 2000);
  stopMotion();

  delay(500);

  // go to the matchloader on the left of red side
  goToPose(first_matchload_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(first_matchload_lineup, 12.0, 4000);
  intakeStop();
  setGoToPoseVelocity(MAX_VELOCITY / 3.0);
  delay(150);
  goToPose(first_matchload, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  intakeBack();
  waitForGoToPose(first_matchload, 2.0, 1100);
  deployBackIntakeArms();
  stopMotion();

  delay(2000);
  hoodRaise();
  delay(500);
  intakeFront();

  // score all blocks in robot
  goToPose(left_goal_front, MAX_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY);
  delay(500);
  retractBackIntakeArms();
  waitForGoToPose(left_goal_front, 2.0, 2000);
  stopMotion();
  delay(250);
  hoodOpenDoor();
  intakeFront();

  delay(2000);

  goToPose(first_matchload, MAX_VELOCITY / 4.0, MAX_ANGULAR_VELOCITY);
  delay(300);
  intakeStop();
  hoodCloseDoor();
  stopMotion();
  delay(150);

  // grab park zone blocks
  goToPose(park_zone_blocks_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY / 4.0);
  waitForGoToPose(park_zone_blocks_lineup, 12.0, 3000);
  setGoToPoseVelocity(MAX_VELOCITY / 5.0);
  waitForGoToPose(park_zone_blocks_lineup, 2.0, 1000);
  goToPose(park_zone_blocks, MAX_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY);
  retractRake();
  waitForGoToPose(park_zone_blocks, 12.0, 1000);
  setGoToPoseVelocity(MAX_VELOCITY / 10.0);
  waitForGoToPose(park_zone_blocks, 2.0, 1500);
  deployRake();
  delay(500);
  goToPose(park_zone_blocks_lineup, MAX_VELOCITY / 4.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(park_zone_blocks_lineup, 2.0, 1000);

  delay(150);

  // grab 2 blocks on the left wall
  goToPose(front_left_wall_blocks_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  intakeFront();
  waitForGoToPose(front_left_wall_blocks_lineup, 6.0, 3000);
  retractRake();
  hoodLower();
  goToPose(front_left_wall_blocks, MAX_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(front_left_wall_blocks, 3.0, 1000);
  stopMotion();
  delay(250);
  deployRake();
  goToPose(front_left_wall_blocks_lineup, MAX_VELOCITY / 5.0,
           MAX_ANGULAR_VELOCITY);
  waitForGoToPose(front_left_wall_blocks_lineup, 6.0, 1000);

  // grab block 7 for the top middle goal
  goToPose(middle_blocks_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(middle_blocks_lineup, 8.0, 4000);
  goToPose(middle_blocks_lineup_2, MAX_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(middle_blocks_lineup_2, 12.0, 1000);
  setGoToPoseVelocity(MAX_VELOCITY / 8.0);
  waitForGoToPose(middle_blocks_lineup_2, 2.0, 4000);
  stopMotion();
  retractRake();
  delay(250);
  goToPose(middle_blocks_end, MAX_VELOCITY / 8.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(middle_blocks_end, 2.0, 2000);
  stopMotion();

  delay(500);

  // block lower middle goal for other bot
  goToPose(middle_top_goal_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(middle_top_goal_lineup, 2.0, 1000);
  intakeStop();
  goToPose(middle_lower_goal_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(middle_lower_goal_lineup, 2.0, 2000);
  goToPose(middle_lower_goal, MAX_VELOCITY / 10.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(middle_lower_goal, 2.0, 2000);
  stopMotion();

  delayUntil(start_time + 32000);

  goToPose(middle_lower_goal_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(middle_lower_goal_lineup, 4.0, 2000);

  // score on middle goal top
  startColorSort(alliance::EAlliance::BLUE);
  goToPose(middle_top_goal_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(middle_top_goal_lineup, 2.0, 4000);
  hoodBumpUp();
  goToPose(middle_top_goal, MAX_VELOCITY / 8.0, MAX_ANGULAR_VELOCITY);
  delay(250);
  setGoToPoseVelocity(MAX_VELOCITY / 10.0);
  waitForGoToPose(middle_top_goal, 2.0, 3000);
  stopMotion();
  hoodOpenDoor();
  intakeFront();
  hoodSetVoltage(6.0);

  delay(1200);
  hoodSetVoltage(9.0);
  delay(1200);
  hoodRaise();
  pauseColorSort();

  goToPose(middle_top_goal_lineup, MAX_VELOCITY / 4.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(middle_top_goal_lineup, 2.0, 1000);
  intakeStop();
  hoodLower();
  hoodCloseDoor();

  // left wall blocks on the far side
  startColorSort(alliance::EAlliance::RED);
  intakeFront();
  goToPose(back_left_wall_blocks_lineup_1, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(back_left_wall_blocks_lineup_1, 16.0, 5000);
  goToPose(back_left_wall_blocks_lineup_2, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(back_left_wall_blocks_lineup_2, 2.0, 2000);
  goToPose(back_left_wall_blocks, MAX_VELOCITY / 4.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(back_left_wall_blocks, 16.0, 2000);
  setGoToPoseVelocity(MAX_VELOCITY / 8.0);
  waitForGoToPose(back_left_wall_blocks, 2.0, 1500);
  stopMotion();
  deployRake();
  delay(250);
  goToPose(back_left_wall_blocks_lineup_2, MAX_VELOCITY / 6.0,
           MAX_ANGULAR_VELOCITY);
  delay(500);
  retractRake();
  goToPose(back_left_wall_blocks, MAX_VELOCITY / 8.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(back_left_wall_blocks, 5.0, 1000);
  delay(100);
  pauseColorSort();

  // back matchloader
  goToPose(second_matchload_lineup, MAX_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(second_matchload_lineup, 12.0, 1500);
  setGoToPoseVelocity(MAX_VELOCITY / 8.0);
  waitForGoToPose(second_matchload_lineup, 2.0, 2500);
  goToPose(second_matchload, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(second_matchload, 6.0, 1500);
  setGoToPoseVelocity(MAX_VELOCITY / 4.0);
  waitForGoToPose(second_matchload, 2.0, 750);
  deployBackIntakeArms();
  intakeBack();

  delay(2000);
  hoodRaise();
  delay(250);
  intakeFront();

  // score on long goal
  goToPose(left_goal_back, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(left_goal_back, 12.0, 2000);
  retractBackIntakeArms();
  setGoToPoseVelocity(MAX_VELOCITY / 8.0);
  waitForGoToPose(left_goal_back, 2.0, 1250);
  hoodOpenDoor();
  intakeFront();

  delay(2250);

  // back up from goal
  goToPose(second_matchload_lineup, MAX_VELOCITY / 6.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(second_matchload_lineup, 2.0, 2000);
  hoodLower();
  hoodCloseDoor();
  intakeStop();
  stopMotion();
  delay(250);

  // go to the park zone
  goToPose(park_lineup_1, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(park_lineup_1, 6.0, 2000);
  goToPose(park_lineup_2, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(park_lineup_2, 10.0, 2000);
  goToPose(park_zone, MAX_VELOCITY / 1.5, MAX_ANGULAR_VELOCITY);

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