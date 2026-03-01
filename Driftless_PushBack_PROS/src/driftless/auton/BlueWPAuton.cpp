#include "driftless/auton/BlueWPAuton.hpp"

namespace driftless::auton {
BlueWPAuton::BlueWPAuton() : AAuton("Blue_Win_Point") {}

void BlueWPAuton::init(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
}

void BlueWPAuton::run(
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
  setOdomPosition(56.0, 20.0, 0.0);
  startColorSort(m_alliance->getAlliance());
  intakeFront();

  // go to matchloader
  intakeFront();
  goToPoint(matchload_lineup, MAX_VELOCITY);
  delay(750);
  setGoToPointVelocity(MAX_VELOCITY / 1.5);
  delay(200);
  goToPoint(matchload, MAX_VELOCITY / 1.5);
  waitForGoToPoint(matchload, 2.0, 1500);
  deployBackIntakeArms();
  intakeBackToHood();

  delay(450);
  hoodRaise();

  delay(175);
  retractBackIntakeArms();

  // score blocks from match loader
  goToPoint(long_goal, MAX_VELOCITY / 1.5);
  delay(200);
  outtakeFront();
  delay(200);
  intakeFront();
  waitForGoToPoint(long_goal, 10.0, 1250);
  intakeFront();

  waitForGoToPoint(long_goal, 2.0, 750);
  hoodOpenDoor();

  delay(1000);

  // go descore
  goToPoint(descore_lineup, MAX_VELOCITY / 2.0);
  middleDescore();
  delay(100);
  intakeStop();
  waitForGoToPoint(descore_lineup, 1.0, 2000);

  goToPoint(end_descore, MAX_VELOCITY / 3.0);
  waitForGoToPoint(end_descore, 2.5, 3000);

  // grab two blocks under goal
  deployDescore();
  goToPoint(two_blocks_lineup, MAX_VELOCITY / 2.0);
  waitForGoToPoint(two_blocks_lineup, 1.5, 2000);

  hoodLower();
  hoodCloseDoor();
  retractDescore();
  intakeFront();

  goToPoint(two_blocks_lineup_2, MAX_VELOCITY / 2.0);
  waitForGoToPoint(two_blocks_lineup_2, 1.5, 2000);

  goToPoint(two_blocks, MAX_VELOCITY / 3.0);
  waitForGoToPoint(two_blocks, 1.0, 3000);

  // go middle 6 blocks
  goToPoint(line_blocks_lineup, MAX_VELOCITY / 2.0);
  waitForGoToPoint(line_blocks_lineup, 1.5, 2500);

  goToPoint(line_blocks_lineup_2, MAX_VELOCITY / 5.0);
  waitForGoToPoint(line_blocks_lineup_2, 1.0, 1000);

  goToPoint(line_blocks, MAX_VELOCITY / 2.5);
  waitForGoToPoint(line_blocks, 1.0, 3500);

  // go bottom goal
  goToPoint(bottom_goal_1, MAX_VELOCITY);
  waitForGoToPoint(bottom_goal_1, 5.0, 1750);
  goToPoint(bottom_goal_2, MAX_VELOCITY / 2);
  waitForGoToPoint(bottom_goal_2, 1.0, 400);
  delay(100);

  hoodSetVoltage(-10.0);
  outtakeFront();

  delay(2300);

  // go matchload again
  intakeStop();
  goToPoint(matchload_lineup_2, MAX_VELOCITY);
  waitForGoToPoint(matchload_lineup_2, 4, 3500);
  goToPoint(matchload_2, MAX_VELOCITY / 1.5);
  waitForGoToPoint(matchload_2, 2.0, 1250);
  deployBackIntakeArms();
  intakeBack();

  delay(2000);

  // go top goal
  goToPoint(top_goal_lineup, MAX_VELOCITY);
  delay(200);
  retractBackIntakeArms();
  intakeStop();
  waitForGoToPoint(top_goal_lineup, 3.0, 2000);
  hoodBumpUp();
  goToPoint(top_goal, MAX_VELOCITY / 3);
  waitForGoToPoint(top_goal, 2.0, 2000);
  hoodOpenDoor();
  intakeFront();
  hoodSetVoltage(6.0);

  delay(1000);
  hoodSetVoltage(9.0);
  delay(1000);
  hoodRaise();

  // go descore
  goToPoint(descore_2_lineup, MAX_VELOCITY / 2.0);
  waitForGoToPoint(descore_2_lineup, 1.0, 2000);

  middleDescore();

  goToPoint(descore_2, MAX_VELOCITY / 2.0);
  waitForGoToPoint(descore_2, 1.0, 2000);

  goToPoint(descore_2_end, MAX_VELOCITY / 3.0);
  waitForGoToPoint(descore_2_end, 1.0, 2000);

  stopMotion();

  // leave at end
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 8, "Runtime: %7.2f",
                      (getTime() - start_time) / 1000.0);
}
}  // namespace driftless::auton