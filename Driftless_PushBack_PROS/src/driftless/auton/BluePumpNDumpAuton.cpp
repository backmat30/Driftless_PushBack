#include "driftless/auton/BluePumpNDumpAuton.hpp"

namespace driftless::auton {
BluePumpNDumpAuton::BluePumpNDumpAuton() : AAuton("Blue Pump N' Dump TM") {}

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
  hoodRaise();

  // go to matchloader
  goToPoint(matchload_lineup, MAX_VELOCITY);
  delay(750);
  setGoToPointVelocity(MAX_VELOCITY / 3.0);
  delay(200);
  goToPoint(matchload, MAX_VELOCITY / 2.0);
  waitForGoToPoint(matchload, 2.0, 1500);
  hoodLower();
  deployBackIntakeArms();
  intakeBackToHood();

  delay(1750);
  intakeStop();

  // score blocks from match loader
  goToPoint(long_goal, MAX_VELOCITY);
  delay(500);
  hoodRaise();
  retractBackIntakeArms();
  setGoToPointVelocity(MAX_VELOCITY / 3.0);
  waitForGoToPoint(long_goal, 2.0, 1000);
  hoodOpenDoor();
  intakeBackToHood();

  // leave at end
  stopMotion();
  intakeFront();
  hoodBumpUp();
  hoodOpenDoor();
  retractBackIntakeArms();
  retractRake();
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 8, "Runtime: %7.2f",
                      (getTime() - start_time) / 1000.0);
}
}  // namespace driftless::auton