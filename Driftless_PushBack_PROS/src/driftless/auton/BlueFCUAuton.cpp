#include "driftless/auton/BlueFCUAuton.hpp"

namespace driftless::auton {
BlueFCUAuton::BlueFCUAuton() : AAuton("Blue_FCU") {}

void BlueFCUAuton::init(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
}

void BlueFCUAuton::run(
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
  waitForGoToPoint(matchload, 2.0, 1000);
  deployBackIntakeArms();
  intakeBackToHood();

  delay(450);
  hoodRaise();

  delay(210);
  retractBackIntakeArms();

  // score blocks from match loader
  goToPoint(long_goal, MAX_VELOCITY / 1.5);
  delay(200);
  outtakeFront();
  delay(200);
  intakeFront();
  waitForGoToPoint(long_goal, 10.0, 1250);
  intakeFront();
  setGoToPointVelocity(MAX_VELOCITY / 2.0);

  waitForGoToPoint(long_goal, 2.0, 750);
  hoodOpenDoor();

  delay(1000);

  // go descore
  goToPoint(descore_lineup, MAX_VELOCITY / 2.0);
  delay(100);
  intakeStop();
  waitForGoToPoint(descore_lineup, 1.0, 2000);

  // grab two blocks under goal
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

  goToPoint(line_blocks, MAX_VELOCITY / 3.5);
  waitForGoToPoint(line_blocks, 1.0, 3500);

  // go bottom goal
  goToPoint(bottom_goal_lineup, MAX_VELOCITY / 2);
  waitForGoToPoint(bottom_goal_lineup, 5.0, 1750);
  goToPoint(bottom_goal_lineup_2, MAX_VELOCITY / 2);
  waitForGoToPoint(bottom_goal_lineup_2, 1.0, 1750);
  goToPoint(bottom_goal, MAX_VELOCITY / 2.0);
  waitForGoToPoint(bottom_goal, 1.0, 600);
  stopMotion();
  delay(1000);

  hoodSetVoltage(-10.0);
  outtakeFront(10.0);

  delay(3000);

  // go matchload again
  intakeStop();
  goToPoint(matchload_lineup_2, MAX_VELOCITY);
  waitForGoToPoint(matchload_lineup_2, 4, 3500);
  goToPoint(matchload_2, MAX_VELOCITY / 1.5);
  waitForGoToPoint(matchload_2, 2.0, 1250);
  deployBackIntakeArms();
  intakeBack();

  delay(2500);

  // go top goal
  goToPoint(top_goal_lineup, MAX_VELOCITY);
  delay(200);
  retractBackIntakeArms();
  intakeStop();
  waitForGoToPoint(top_goal_lineup, 3.0, 2000);
  hoodBumpUp();
  goToPoint(top_goal, MAX_VELOCITY / 3);
  waitForGoToPoint(top_goal, 2.0, 2000);
  delayUntil(start_time + 26000);
  hoodOpenDoor();
  intakeFront();
  hoodSetVoltage(6.0);
  

  delay(1000);
  hoodSetVoltage(9.0);
  delay(1000);
  hoodRaise();
  delay(250);
  hoodCloseDoor();
  intakeStop();

  goToPoint(descore_2_lineup, MAX_VELOCITY);
  middleDescore();
  waitForGoToPoint(descore_2_lineup, 2.0, 2000);
  goToPoint(descore_2, MAX_VELOCITY);
  waitForGoToPoint(descore_2, 1.0, 2000);
  goToPoint(descore_2_end, MAX_VELOCITY);
  waitForGoToPoint(descore_2_end, 1.0, 1000);

  stopMotion();

  // leave at end
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 8, "Runtime: %7.2f",
                      (getTime() - start_time) / 1000.0);
}
}  // namespace driftless::auton