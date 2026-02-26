#include "driftless/auton/OrangeSkillsAuton.hpp"

namespace driftless::auton {
OrangeSkillsAuton::OrangeSkillsAuton() : AAuton("Orange_Skills") {}

void OrangeSkillsAuton::init(
    std::shared_ptr<robot::Robot>& robot,
    std::shared_ptr<control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
}

void OrangeSkillsAuton::run(
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
  setOdomPosition(88.0, 20.0, M_PI);

  // head to the blues under the long goal
  goToPoint(middle_blues_lineup_1, MAX_VELOCITY);
  waitForGoToPoint(middle_blues_lineup_1, 12.0, 3000);
  setGoToPointVelocity(MAX_VELOCITY / 2.0);
  waitForGoToPoint(middle_blues_lineup_1, 8.0, 3000);
  goToPoint(middle_blues_lineup_2, MAX_VELOCITY);
  waitForGoToPoint(middle_blues_lineup_2, 18.0, 3000);
  setGoToPointVelocity(MAX_VELOCITY / 3.0);
  waitForGoToPoint(middle_blues_lineup_2, 2.0, 2000);
  deployRake();
  intakeFront();
  goToPoint(middle_blues, MAX_VELOCITY / 1.5);
  waitForGoToPoint(middle_blues, 2.0, 3000);

  // go to the first matchload on blue side
  goToPoint(back_matchload_lineup, MAX_VELOCITY);
  waitForGoToPoint(back_matchload_lineup, 8.0, 3000);
  goToPoint(back_matchload_lineup_2, MAX_VELOCITY / 2.0);
  delay(150);
  setGoToPointVelocity(MAX_VELOCITY);
  waitForGoToPoint(back_matchload_lineup_2, 2.0, 3000);
  retractRake();
  goToPoint(back_matchload, MAX_VELOCITY);
  waitForGoToPoint(back_matchload, 16.0, 3000);
  setGoToPointVelocity(MAX_VELOCITY / 3.0);
  waitForGoToPoint(back_matchload, 2.0, 1000);
  stopMotion();
  intakeBack();
  deployBackIntakeArms();

  delay(2250);

  // score on the goal
  hoodRaise();
  delay(250);
  goToPoint(back_long_goal, MAX_VELOCITY / 2.0);
  intakeFront();
  waitForGoToPoint(back_long_goal, 16.0, 1500);
  setGoToPointVelocity(MAX_VELOCITY / 3.0);
  waitForGoToPoint(back_long_goal, 2.0, 1250);
  stopMotion();
  hoodOpenDoor();
  retractBackIntakeArms();

  delay(2000);

  // back up and close hood
  goToPoint(back_matchload, MAX_VELOCITY / 2.0);
  outtakeFront();
  delay(400);
  hoodCloseDoor();
  hoodLower();
  stopMotion();
  delay(250);

  // go to blue park zone for blocks
  startColorSort(alliance::EAlliance::RED);
  goToPoint(park_zone_blocks_lineup, MAX_VELOCITY);
  waitForGoToPoint(park_zone_blocks_lineup, 18.0, 2000);
  setGoToPointVelocity(MAX_VELOCITY / 2.5);
  waitForGoToPoint(park_zone_blocks_lineup, 4.0, 2000);
  goToPoint(park_zone_blocks, MAX_VELOCITY / 1.5);
  waitForGoToPoint(park_zone_blocks, 14.0, 1000);
  setGoToPointVelocity(MAX_VELOCITY / 4.0);
  waitForGoToPoint(park_zone_blocks, 3.0, 2000);
  deployRake();
  intakeFront();
  delay(500);
  goToPoint(park_zone_blocks_lineup, MAX_VELOCITY / 3.0);
  waitForGoToPoint(park_zone_blocks_lineup, 2.0, 1000);

  // grab the red blocks against the wall
  goToPoint(wall_blocks_lineup, MAX_VELOCITY);
  waitForGoToPoint(wall_blocks_lineup, 8.0, 2000);
  goToPoint(wall_blocks_start, MAX_VELOCITY / 1.5);
  waitForGoToPoint(wall_blocks_start, 24.0, 3000);
  retractRake();
  setGoToPointVelocity(MAX_VELOCITY / 3.0);
  waitForGoToPoint(wall_blocks_start, 3.0, 2500);
  deployRake();
  goToPoint(wall_blocks_lineup, MAX_VELOCITY / 3.5);
  waitForGoToPoint(wall_blocks_lineup, 10.0, 750);
  retractRake();
  stopMotion();
  delay(400);
  goToPoint(wall_blocks_start, MAX_VELOCITY / 1.5);
  waitForGoToPoint(wall_blocks_start, 4.0, 3000);
  delay(500);
  goToPoint(wall_blocks_end, MAX_VELOCITY);
  waitForGoToPoint(wall_blocks_end, 4.0, 4000);

  // grab reds under long goal
  turnToHeading(middle_reds.getTheta(), M_PI * 3.0);
  delay(500);
  goToPoint(middle_reds, MAX_VELOCITY / 2.0);
  waitForGoToPoint(middle_reds, 4.0, 1000);

  // deposit blocks in middle goal
  goToPoint(middle_goal_lineup, MAX_VELOCITY);
  waitForGoToPoint(middle_goal_lineup, 2.0, 2000);
  goToPoint(middle_goal, MAX_VELOCITY / 1.5);
  waitForGoToPoint(middle_goal, 10.0, 3000);
  goToPoint(middle_goal, MAX_VELOCITY / 3.5);
  waitForGoToPoint(middle_goal, 2.0, 1000);
  outtakeFront();
  hoodSetVoltage(-12.0);
  pauseColorSort();

  delay(1000);
  intakeFront();
  delay(250);
  hoodRaise();
  hoodSetVoltage(-12.0);
  outtakeFront();
  delay(2000);
  hoodLower();

  // block top middle goal for blue
  startColorSort(alliance::EAlliance::BLUE);
  goToPoint(middle_goal_lineup, MAX_VELOCITY / 4.0);
  delay(1000);
  goToPoint(middle_goal_top_lineup, MAX_VELOCITY);
  waitForGoToPoint(middle_goal_top_lineup, 2.0, 3000);
  hoodBumpUp();
  goToPoint(middle_goal_top, MAX_VELOCITY / 3.0);
  waitForGoToPoint(middle_goal_top, 2.0, 3000);
  stopMotion();
  delay(2000);
  goToPoint(middle_goal_top_lineup, MAX_VELOCITY / 2.0);
  waitForGoToPoint(middle_goal_top_lineup, 2.0, 3000);

  // go to the blue wall blocks
  goToPoint(front_wall_blocks_lineup, MAX_VELOCITY);
  waitForGoToPoint(front_wall_blocks_lineup, 8.0, 4000);
  intakeFront();
  goToPoint(front_wall_blocks_start, MAX_VELOCITY);
  waitForGoToPoint(front_wall_blocks_start, 10.0, 2000);
  setGoToPointVelocity(MAX_VELOCITY / 5.0);
  waitForGoToPoint(front_wall_blocks_start, 2.0, 1500);
  pauseColorSort();
  deployRake();
  delay(250);
  goToPoint(front_wall_blocks_end, MAX_VELOCITY / 2.0);
  waitForGoToPoint(front_wall_blocks_end, 4.0, 1250);
  stopMotion();
  retractRake();
  delay(250);
  goToPoint(front_wall_blocks_start, MAX_VELOCITY / 2.0);
  delay(500);
  stopMotion();
  delay(250);

  // go to front matchloader
  goToPoint(front_match_load_lineup, MAX_VELOCITY);
  waitForGoToPoint(front_match_load_lineup, 4.0, 2000);
  goToPoint(front_match_load, MAX_VELOCITY);
  waitForGoToPoint(front_match_load, 16.0, 4000);
  setGoToPointVelocity(MAX_VELOCITY / 2.0);
  waitForGoToPoint(front_match_load, 2.0, 1750);
  intakeBack();
  deployBackIntakeArms();

  delay(2250);

  // go to goal
  goToPoint(front_right_long_goal, MAX_VELOCITY / 2.5);
  intakeFront();
  delay(300);
  retractBackIntakeArms();
  hoodRaise();
  waitForGoToPoint(front_right_long_goal, 2.0, 2500);
  stopMotion();
  hoodOpenDoor();

  delay(3000);

  // park
  goToPoint(park_lineup, MAX_VELOCITY);
  hoodLower();
  waitForGoToPoint(park_lineup, 4.0, 3000);
  hoodCloseDoor();
  intakeStop();
  goToPoint(park, MAX_VELOCITY / 1.25);

  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 8, "Runtime: %7.2f",
                      (getTime() - start_time) / 1000.0);
  delayUntil(start_time + 59500);
  hoodRaise();
  delayUntil(start_time + 60000);

  // leave at end
  stopMotion();
  // intakeFront();
  // hoodBumpUp();
  hoodOpenDoor();
  retractBackIntakeArms();
  retractRake();
}
}  // namespace driftless::auton