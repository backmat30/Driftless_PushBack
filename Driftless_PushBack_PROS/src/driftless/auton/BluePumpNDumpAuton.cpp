#include "driftless/auton/BluePumpNDumpAuton.hpp"

namespace driftless::auton {
BluePumpNDumpAuton::BluePumpNDumpAuton() : AAuton("Blue_PumpDump") {}

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
  setOdomPosition(56.0, 20.0, 0.0);
  startColorSort(m_alliance->getAlliance());
  intakeFront();

  // go to matchloader
  goToPose(matchload_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  delay(750);
  setGoToPoseVelocity(MAX_VELOCITY / 2.0);
  delay(200);
  goToPose(matchload, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(matchload, 2.0, 1500);
  deployBackIntakeArms();
  intakeBackToHood();

  delay(450);
  hoodRaise();
  delay(175);
  intakeStop();
  retractBackIntakeArms();

  // score blocks from match loader
  intakeFront();
  goToPose(long_goal, MAX_VELOCITY / 1.25, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(long_goal, 12.0, 1000);
  setGoToPoseVelocity(MAX_VELOCITY / 3.5);
  waitForGoToPose(long_goal, 2.0, 1000);
  hoodOpenDoor();

  delay(1250);

  // go back to match loader
  intakeStop();
  goToPose(long_goal_to_matchload, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(long_goal_to_matchload, 16.0, 1500);
  setGoToPoseVelocity(MAX_VELOCITY / 2.5);
  waitForGoToPose(long_goal_to_matchload, 6.0, 2000);
  goToPose(matchload, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  intakeBack();
  waitForGoToPose(matchload, 12.0, 2000);
  hoodLower();
  hoodCloseDoor();
  setGoToPoseVelocity(MAX_VELOCITY / 2.5);
  waitForGoToPose(matchload, 2.0, 2000);
  deployBackIntakeArms();

  delay(3300);
  hoodRaise();
  delay(200);

  // score on long goal again
  intakeFront();
  retractBackIntakeArms();
  goToPose(long_goal_2, MAX_VELOCITY / 1.25, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(long_goal_2, 12.0, 1000);
  setGoToPoseVelocity(MAX_VELOCITY / 3.5);
  waitForGoToPose(long_goal_2, 2.0, 1000);
  hoodOpenDoor();
  intakeFront();

  // block goal until almost end of auton
  delay(2500);
  goToPose(matchload_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  delay(250);
  hoodCloseDoor();
  intakeStop();
  goToPose(long_goal, MAX_VELOCITY / 1.25, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(long_goal, 2.0, 2000);
  stopMotion();
  delayUntil(start_time + 19000);

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