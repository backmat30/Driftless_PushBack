#include "driftless/auton/OrangePumpNDumpAuton.hpp"

namespace driftless::auton {
OrangePumpNDumpAuton::OrangePumpNDumpAuton() : AAuton("Orange_PumpDump") {}

void OrangePumpNDumpAuton::init(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
}

void OrangePumpNDumpAuton::run(
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
  setOdomPosition(96.5, 20.0, M_PI);
  startColorSort(m_alliance->getAlliance());
  intakeFront();

  // go to matchloader
  goToPoint(matchload_lineup, MAX_VELOCITY);
  delay(750);
  setGoToPointVelocity(MAX_VELOCITY / 2.0);
  delay(200);
  goToPoint(matchload, MAX_VELOCITY / 2.0);
  waitForGoToPoint(matchload, 2.0, 1500);
  deployBackIntakeArms();
  intakeBackToHood();

  delay(800);
  intakeStop();
  retractBackIntakeArms();

  // score blocks from match loader
  goToPoint(long_goal_lineup, MAX_VELOCITY / 1.5);
  waitForGoToPoint(long_goal_lineup, 8.0, 1500);
  intakeFront();
  goToPoint(long_goal_lineup_2, MAX_VELOCITY / 1.75);
  waitForGoToPoint(long_goal_lineup_2, 4.0, 1500);
  hoodRaise();
  goToPoint(long_goal, MAX_VELOCITY / 3.5);
  waitForGoToPoint(long_goal, 2.0, 2000);
  hoodOpenDoor();

  delay(1250);

  // go back to match loader
  intakeStop();
  goToPoint(long_goal_to_matchload, MAX_VELOCITY);
  waitForGoToPoint(long_goal_to_matchload, 16.0, 1500);
  setGoToPointVelocity(MAX_VELOCITY / 2.5);
  waitForGoToPoint(long_goal_to_matchload, 6.0, 2000);
  goToPoint(matchload, MAX_VELOCITY);
  intakeBack();
  waitForGoToPoint(matchload, 12.0, 2000);
  hoodLower();
  hoodCloseDoor();
  setGoToPointVelocity(MAX_VELOCITY / 2.5);
  waitForGoToPoint(matchload, 2.0, 2000);
  deployBackIntakeArms();

  delay(3500);

  // score on long goal again
  intakeStop();
  goToPoint(long_goal_lineup, MAX_VELOCITY / 1.5);
  waitForGoToPoint(long_goal_lineup, 6.0, 2000);
  goToPoint(long_goal_2, MAX_VELOCITY / 1.25);
  waitForGoToPoint(long_goal_2, 12.0, 1000);
  hoodRaise();
  retractBackIntakeArms();
  setGoToPointVelocity(MAX_VELOCITY / 3.5);
  waitForGoToPoint(long_goal_2, 2.0, 1000);
  hoodOpenDoor();
  intakeFront();

  // block goal until almost end of auton
  delay(2500);
  goToPoint(matchload_lineup, MAX_VELOCITY / 2.0);
  delay(250);
  hoodCloseDoor();
  intakeStop();
  goToPoint(long_goal, MAX_VELOCITY / 1.25);
  waitForGoToPoint(long_goal, 2.0, 2000);
  stopMotion();
  delayUntil(start_time + 27000);

  // descore
  goToPoint(matchload_lineup, MAX_VELOCITY / 2.0);
  deployDescore();
  waitForGoToPoint(matchload_lineup, 6.0, 1000);
  goToPoint(descore_lineup, MAX_VELOCITY);
  waitForGoToPoint(descore_lineup, 2.0, 2000);
  goToPoint(start_descore, MAX_VELOCITY / 2.0);
  waitForGoToPoint(start_descore, 2.0, 2000);
  stopMotion();
  retractDescore();
  delay(250);
  goToPoint(end_descore, MAX_VELOCITY / 2.0);
  waitForGoToPoint(end_descore, 2.0, 3000);
  stopMotion();

  // leave at end
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 8, "Runtime: %7.2f",
                      (getTime() - start_time) / 1000.0);
}
}  // namespace driftless::auton