#include "driftless/auton/OrangePumpNDumpAuton.hpp"

namespace driftless::auton {
OrangePumpNDumpAuton::OrangePumpNDumpAuton() : AAuton("O_10B_L") {}

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
  setOdomPosition(88.0, 20.0, M_PI);
  startColorSort(m_alliance->getAlliance());
  intakeFront();

  // go to front matchloader
  goToPose(match_load_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 80.0);
  waitForGoToPose(match_load_lineup, 2.0, 2000);
  goToPose(match_load, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 20.0);
  waitForGoToPose(match_load, 2.0, 2000);
  intakeBack();
  deployBackIntakeArms();

  delay(450);
  hoodRaise();
  delay(300);

  // score blocks from match loader
  goToPose(long_goal, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 30.0);
  delay(100);
  intakeStop();
  retractBackIntakeArms();
  delay(150);
  waitForGoToPose(long_goal, 12.0, 1000);
  intakeFront();
  waitForGoToPose(long_goal, 2.0, 750);
  hoodOpenDoor();

  delay(1000);

  // go back to match loader
  intakeBack();
  goToPose(match_load, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(match_load, 2.0, 3000);
  hoodCloseDoor();
  deployBackIntakeArms();

  delay(3500);

  // score on long goal again
  goToPose(long_goal, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 30.0);
  delay(100);
  intakeStop();
  retractBackIntakeArms();
  delay(150);
  waitForGoToPose(long_goal, 12.0, 1000);
  intakeFront();
  waitForGoToPose(long_goal, 2.0, 750);
  hoodOpenDoor();

  delay(2500);

  // block goal until almost end of auton
  delay(2500);
  goToPose(match_load_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  delay(250);
  hoodCloseDoor();
  intakeStop();
  goToPose(long_goal, MAX_VELOCITY / 2.5, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(long_goal, 2.0, 2000);
  stopMotion();
  delayUntil(start_time + 19000);

  // descore
  goToPose(descore_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  middleDescore();
  delay(100);
  intakeStop();
  waitForGoToPose(descore_lineup, 1.0, 2000);

  goToPose(end_descore, MAX_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(end_descore, 2.5, 3000);
  stopMotion();

  // leave at end
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 8, "Runtime: %7.2f",
                      (getTime() - start_time) / 1000.0);
}
}  // namespace driftless::auton