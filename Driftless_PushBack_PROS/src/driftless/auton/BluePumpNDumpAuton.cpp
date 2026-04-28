#include "driftless/auton/BluePumpNDumpAuton.hpp"

namespace driftless::auton {
BluePumpNDumpAuton::BluePumpNDumpAuton() : AAuton("B_10B_R") {}

void BluePumpNDumpAuton::init(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
}

void BluePumpNDumpAuton::run(
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

  //  go to matchloader
  intakeFront();
  goToPose(matchload_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 60.0);
  waitForGoToPoint(matchload_lineup, 2.0, 1000);
  goToPose(matchload, MAX_VELOCITY / 1.5, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(matchload, 2.0, 1500);
  deployBackIntakeArms();
  intakeBackToHood();

  delay(400);
  hoodRaise();

  delay(250);

  // score blocks from match loader
  goToPose(long_goal, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(long_goal, 10.0, 1250);
  intakeFront();
  retractBackIntakeArms();

  waitForGoToPose(long_goal, 2.0, 750);
  hoodOpenDoor();

  delay(1250);

  // go back to match loader
  intakeStop();
  goToPose(matchload, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 30.0);
  intakeBack();
  hoodCloseDoor();
  waitForGoToPose(matchload, 2.0, 2000);
  deployBackIntakeArms();

  delay(3300);
  hoodRaise();
  delay(200);

  // score on long goal again
  retractBackIntakeArms();
  goToPose(long_goal, MAX_VELOCITY, MAX_ANGULAR_VELOCITY, 30.0);
  waitForGoToPose(long_goal, 16.0, 1000);
  intakeFront(6.0);
  waitForGoToPose(long_goal, 2.0, 1000);
  hoodOpenDoor();
  intakeFront();

  // block goal until almost end of auton
  delay(2750);
  goToPose(matchload_lineup, MAX_VELOCITY / 4.0, MAX_ANGULAR_VELOCITY);
  delay(500);
  hoodCloseDoor();
  intakeStop();
  // goToPose(long_goal, MAX_VELOCITY / 2.5, MAX_ANGULAR_VELOCITY);
  // waitForGoToPose(long_goal, 2.0, 2000);
  // stopMotion();
  // delayUntil(start_time + 16000);

  // descore
  goToPose(descore_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  middleDescore();
  delay(100);
  intakeStop();
  waitForGoToPose(descore_lineup, 1.25, 2000);

  goToPose(end_descore, MAX_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(end_descore, 2.0, 3000);
  stopMotion();

  stopMotion();

  // leave at end
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 8, "Runtime: %7.2f",
                      (getTime() - start_time) / 1000.0);
}
}  // namespace driftless::auton