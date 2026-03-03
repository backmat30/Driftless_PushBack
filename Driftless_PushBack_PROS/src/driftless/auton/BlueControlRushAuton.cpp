#include "driftless/auton/BlueControlRushAuton.hpp"

namespace driftless::auton {
BlueControlRushAuton::BlueControlRushAuton() : AAuton("Blue_Control_Rush") {}

void BlueControlRushAuton::init(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
}

void BlueControlRushAuton::run(
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
  goToPose(matchload_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  delay(750);
  setGoToPoseVelocity(MAX_VELOCITY / 1.5);
  delay(200);
  goToPose(matchload, MAX_VELOCITY / 1.5, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(matchload, 2.0, 1500);
  deployBackIntakeArms();
  intakeBackToHood();

  delay(450);
  hoodRaise();

  delay(225);
  retractBackIntakeArms();

  // score blocks from match loader
  goToPose(long_goal, MAX_VELOCITY / 1.5, MAX_ANGULAR_VELOCITY);
  delay(200);
  outtakeFront();
  delay(200);
  intakeFront();
  waitForGoToPose(long_goal, 10.0, 1250);
  intakeFront();

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

  stopMotion();

  // leave at end
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 8, "Runtime: %7.2f",
                      (getTime() - start_time) / 1000.0);
}
}  // namespace driftless::auton