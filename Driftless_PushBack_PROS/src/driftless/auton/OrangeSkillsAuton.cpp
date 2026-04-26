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
  setOdomPosition(88.0, 20.5, M_PI);

  // go to front matchloader
  goToPose(front_match_load_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 40.0);
  waitForGoToPose(front_match_load_lineup, 2.0, 2000);
  goToPose(front_match_load, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY, 20.0);
  waitForGoToPose(front_match_load, 2.0, 2000);
  stopMotion();
  intakeBack();
  deployBackIntakeArms();

  delay(1500);
  hoodRaise();
  delay(750);

  // go to the blue wall blocks
  goToPose(front_wall_blocks_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 40.0);
  intakeFront();
  waitForGoToPose(front_wall_blocks_lineup, 8.0, 4000);
  retractBackIntakeArms();
  goToPose(front_wall_blocks, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 20.0);
  waitForGoToPose(front_wall_blocks, 2.0, 1500);
  pauseColorSort();
  deployRake();
  delay(250);
  goToPose(front_wall_blocks_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY,
           30.0);
  waitForGoToPose(front_wall_blocks_lineup, 4.0, 1250);
  stopMotion();
  retractRake();
  delay(250);
  goToPose(front_wall_blocks, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 20.0);
  delay(500);
  stopMotion();
  delay(500);

  // go to blue side to score
  goToPose(far_goal_lineup_1, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 40.0);
  waitForGoToPose(far_goal_lineup_1, 2.0, 5000);
  intakeStop();
  goToPose(back_matchload_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(back_matchload_lineup, 2.0, 2000);
  goToPose(back_long_goal, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(back_long_goal, 2.0, 2000);
  stopMotion();
  hoodOpenDoor();
  intakeFront();
  delay(3000);
  goToPose(back_matchload_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY,
           20.0);
  delay(250);
  hoodCloseDoor();
  hoodLower();
  delay(500);

  // go to blue park zone for blocks
  startColorSort(alliance::EAlliance::RED);
  goToPose(park_zone_blocks_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY / 3.0,
           40.0);
  waitForGoToPose(park_zone_blocks_lineup, 18.0, 2000);
  hoodLower();
  intakeStop();
  hoodCloseDoor();
  waitForGoToPose(park_zone_blocks_lineup, 2.0, 3000);
  goToPose(park_zone_blocks, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY, 10.0);
  waitForGoToPose(park_zone_blocks, 2.0, 2000);
  deployRake();
  intakeFront();
  stopMotion();
  delay(500);
  goToPose(park_zone_blocks_lineup, MAX_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(park_zone_blocks_lineup, 2.0, 1000);

  // grab the red blocks against the wall
  goToPose(wall_blocks_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY / 2.0);
  waitForGoToPose(wall_blocks_lineup, 8.0, 2000);
  goToPose(wall_blocks, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(wall_blocks, 24.0, 3000);
  retractRake();
  waitForGoToPose(wall_blocks, 2.0, 2500);
  deployRake();
  delay(500);
  goToPose(wall_blocks_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY, 40.0);
  waitForGoToPose(wall_blocks_lineup, 10.0, 750);
  retractRake();
  stopMotion();
  delay(400);
  goToPose(wall_blocks, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(wall_blocks, 4.0, 3000);
  delay(500);

  // grab reds under long goal
  goToPose(middle_reds_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY / 2.0, 40.0);
  waitForGoToPose(middle_reds_lineup, 4.0, 4000);
  goToPose(middle_reds, MAX_VELOCITY / 4.0, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(middle_reds, 1.0, 2000);
  deployRake();
  goToPose(middle_reds_lineup, MAX_VELOCITY / 4.0, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(middle_reds_lineup, 1.0, 2000);

  // deposit blocks in middle goal
  goToPose(middle_goal_lineup_1, MAX_VELOCITY, MAX_ANGULAR_VELOCITY / 6.0,
           40.0);
  waitForGoToPose(middle_goal_lineup_1, 3.0, 2000);
  retractRake();
  goToPose(middle_goal_lineup_2, MAX_VELOCITY, MAX_ANGULAR_VELOCITY / 6.0,
           40.0);
  waitForGoToPose(middle_goal_lineup_2, 3.0, 4000);
  goToPose(middle_goal, MAX_VELOCITY / 1.5, MAX_ANGULAR_VELOCITY, 20.0);
  waitForGoToPose(middle_goal, 2.0, 3000);
  stopMotion();
  delayUntil(start_time + 33000);
  hoodSetVoltage(-12.0);
  outtakeFront();
  delay(750);
  intakeFront();
  delay(100);
  outtakeFront();
  hoodSetVoltage(-12.0);
  delay(1500);
  outtakeFront(6.0);
  delay(250);

  // go to the matchload on blue side
  startColorSort(alliance::EAlliance::BLUE);
  goToPose(middle_goal_lineup_2, MAX_VELOCITY / 4.0, MAX_ANGULAR_VELOCITY,
           30.0);
  delay(1000);
  goToPose(middle_goal_top_lineup, MAX_VELOCITY / 4.0, MAX_ANGULAR_VELOCITY,
           40.0);
  intakeFront();
  waitForGoToPose(middle_goal_top_lineup, 2.0, 3000);
  goToPose(back_matchload_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY / 3.0,
           60.0);
  delay(500);
  setGoToPoseAngularVelocity(MAX_ANGULAR_VELOCITY);
  waitForGoToPose(back_matchload_lineup, 2.0, 3000);
  pauseColorSort();
  goToPose(back_matchload, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(back_matchload, 2.0, 1500);
  stopMotion();
  intakeBack();
  deployBackIntakeArms();
  hoodRaise();
  delay(2250);

  // score on the goal
  goToPose(back_long_goal, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 20.0);
  intakeFront();
  delay(500);
  intakeStop();
  hoodOpenDoor();
  waitForGoToPose(back_long_goal, 2.0, 1250);
  intakeFront();
  stopMotion();
  retractBackIntakeArms();

  delay(1000);
  hoodSetVoltage(9.0);
  delay(1500);

  // back up and close hood
  goToPose(back_matchload, MAX_VELOCITY / 4.0, MAX_ANGULAR_VELOCITY);
  delay(500);
  stopMotion();
  hoodCloseDoor();
  hoodLower();
  intakeStop();
  delay(500);

  // park
  intakeFront();
  goToPose(wall_blocks, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(wall_blocks, 2.0, 1500);
  goToPose(park_lineup_1, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(park_lineup_1, 4.0, 4000);
  goToPose(park_lineup_2, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 60.0);
  waitForGoToPose(park_lineup_2, 8.0, 4000);
  intakeStop();
  goToPose(park, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 50.0);

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