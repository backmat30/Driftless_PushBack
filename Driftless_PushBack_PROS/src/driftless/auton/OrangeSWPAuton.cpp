#include "driftless/auton/OrangeSWPAuton.hpp"

namespace driftless::auton {
OrangeSWPAuton::OrangeSWPAuton() : AAuton("O_SWP_B->T") {}

void OrangeSWPAuton::init(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
}

void OrangeSWPAuton::run(
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

  uint32_t start_time{getTime()};
  setOdomPosition(88.0, 20.0, M_PI);
  startColorSort(m_alliance->getAlliance());

  // ROUTE GOES HERE
  intakeFront();

  //  go to matchloader
  intakeFront();
  goToPose(match_load_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 60.0);
  waitForGoToPoint(match_load_lineup, 2.0, 1000);
  goToPose(match_load, MAX_VELOCITY / 1.5, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(match_load, 2.0, 1500);
  deployBackIntakeArms();
  intakeBackToHood();

  delay(400);
  hoodRaise();

  delay(225);

  // score blocks from match loader
  goToPose(long_goal, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(long_goal, 16.0, 1250);
  intakeFront();
  retractBackIntakeArms();

  waitForGoToPose(long_goal, 2.0, 750);
  hoodOpenDoor();

  delay(1000);

  // go descore
  goToPose(descore_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  middleDescore();
  delay(100);
  intakeStop();
  waitForGoToPose(descore_lineup, 1.0, 2000);

  goToPose(end_descore, MAX_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(end_descore, 2.5, 3000);

  // grab two blocks under goal
  deployDescore();
  goToPose(two_blocks_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(two_blocks_lineup, 1.5, 2000);

  hoodLower();
  hoodCloseDoor();
  retractDescore();
  intakeFront();
  delay(200);

  goToPose(two_blocks_lineup_2, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(two_blocks_lineup_2, 1.5, 2000);

  goToPose(two_blocks, MAX_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(two_blocks, 1.0, 3000);

  // go middle 6 blocks
  goToPose(line_blocks_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY / 3.0);
  waitForGoToPose(line_blocks_lineup, 1.5, 2500);

  goToPose(line_blocks_lineup_2, MAX_VELOCITY / 5.0,
           MAX_ANGULAR_VELOCITY / 2.0);
  waitForGoToPose(line_blocks_lineup_2, 1.0, 1000);

  goToPose(line_blocks, MAX_VELOCITY / 5.0, MAX_ANGULAR_VELOCITY / 10.0);
  waitForGoToPose(line_blocks, 1.0, 3500);
  stopMotion();
  delay(1000);

  // go bottom goal
  goToPose(bottom_goal_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(bottom_goal_lineup, 2.0, 1750);
  goToPose(bottom_goal, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY, 20.0);
  waitForGoToPose(bottom_goal, 2.0, 1000);
  stopMotion();
  delay(1000);

  hoodSetVoltage(-12.0);
  outtakeFront(8.0);

  delay(2800);

  // go matchload again
  intakeStop();
  goToPose(match_load_lineup_2, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 40.0);
  waitForGoToPose(match_load_lineup_2, 4, 3500);
  goToPose(match_load, MAX_VELOCITY / 1.5, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(match_load, 2.0, 1500);
  deployBackIntakeArms();
  intakeBack();

  delay(2500);

  // go top goal
  goToPose(top_goal_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 30.0);
  delay(200);
  retractBackIntakeArms();
  intakeFront();
  waitForGoToPose(top_goal_lineup, 3.0, 2000);
  hoodBumpUp();
  goToPose(top_goal, MAX_VELOCITY / 3, MAX_ANGULAR_VELOCITY, 10.0);
  waitForGoToPose(top_goal, 2.0, 2000);
  hoodOpenDoor();
  intakeStop();
  hoodSetVoltage(12.0);
  delay(1000);
  hoodRaise();
  hoodCloseDoor();
  intakeStop();
  hoodSetVoltage(0.0);

  // go descore
  goToPose(descore_2_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(descore_2_lineup, 1.0, 2000);

  middleDescore();

  goToPose(descore_2, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(descore_2, 1.0, 2000);

  goToPose(descore_2_end, MAX_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(descore_2_end, 1.0, 2000);

  stopMotion();

  // leave at end
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 8, "Runtime: %7.2f",
                      (getTime() - start_time) / 1000.0);
}
}  // namespace driftless::auton