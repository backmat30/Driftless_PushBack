#include "driftless/auton/BlueSafeWP.hpp"

namespace driftless::auton {
BlueSafeWP::BlueSafeWP() : AAuton("Blue_Safe_T->B") {}

void BlueSafeWP::init(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
}

void BlueSafeWP::run(
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
  setOdomPosition(58.0, 20.0, M_PI);
  startColorSort(m_alliance->getAlliance());
  intakeFront();

  // go to matchloader
  intakeFront();
  goToPose(matchload_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 60.0);
  waitForGoToPoint(matchload_lineup, 2.0, 1000);
  goToPose(matchload, MAX_VELOCITY / 1.5, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(matchload, 2.0, 1000);
  deployBackIntakeArms();
  intakeBackToHood();

  delay(450);
  hoodRaise();

  delay(300);

  // score blocks from match loader
  goToPose(long_goal, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(long_goal, 16.0, 1250);
  outtakeFront(5.0);
  delay(500);
  intakeFront();

  waitForGoToPose(long_goal, 2.0, 750);
  retractBackIntakeArms();
  hoodOpenDoor();

  delay(1500);

  // go "descore"
  goToPose(descore_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  delay(100);
  intakeStop();
  waitForGoToPose(descore_lineup, 1.0, 2000);
  hoodLower();
  hoodCloseDoor();
  retractDescore();

  // grab two blocks under goal
  goToPose(two_blocks_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(two_blocks_lineup, 1.5, 2000);

  intakeFront();

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

  goToPose(line_blocks, MAX_VELOCITY / 5.0, MAX_ANGULAR_VELOCITY / 2.0);
  waitForGoToPose(line_blocks, 1.0, 3500);

  // go bottom goal
  goToPose(bottom_goal_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(bottom_goal_lineup, 5.0, 1750);
  goToPose(bottom_goal_lineup_2, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 20.0);
  waitForGoToPose(bottom_goal_lineup_2, 2.0, 2500);
  intakeStop();
  goToPose(bottom_goal, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY, 20.0);
  waitForGoToPose(bottom_goal, 2.0, 1000);
  stopMotion();
  delay(1000);

  hoodSetVoltage(-10.0);
  outtakeFront(8.0);

  delay(2800);

  // go matchload again
  intakeStop();
  goToPose(matchload_lineup_2, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(matchload_lineup_2, 4, 3500);
  goToPose(matchload_2, MAX_VELOCITY / 1.5, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(matchload_2, 2.0, 1250);
  deployBackIntakeArms();
  intakeBack();

  delay(2500);

  // go top goal
  goToPose(top_goal_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  delay(200);
  retractBackIntakeArms();
  intakeStop();
  waitForGoToPose(top_goal_lineup, 3.0, 2000);
  hoodBumpUp();
  goToPose(top_goal, MAX_VELOCITY / 3, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(top_goal, 2.0, 2000);
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

  goToPose(descore_2_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  middleDescore();
  waitForGoToPose(descore_2_lineup, 2.0, 2000);
  goToPose(descore_2, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(descore_2, 1.0, 2000);
  goToPose(descore_2_end, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(descore_2_end, 1.0, 1000);

  stopMotion();

  // leave at end
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 8, "Runtime: %7.2f",
                      (getTime() - start_time) / 1000.0);
}
}  // namespace driftless::auton