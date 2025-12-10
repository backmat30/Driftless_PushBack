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
  goToPoint(matchload_lineup, MAX_VELOCITY);
  delay(750);
  setGoToPointVelocity(MAX_VELOCITY / 2.0);
  delay(200);
  goToPoint(matchload, MAX_VELOCITY / 2.0);
  waitForGoToPoint(matchload, 2.0, 1500);
  deployBackIntakeArms();
  intakeBackToHood();

  delay(2000);
  outtakeFront();

  // score blocks from match loader
  goToPoint(long_goal, MAX_VELOCITY / 1.25);
  delay(750);
  intakeFront();
  hoodRaise();
  retractBackIntakeArms();
  setGoToPointVelocity(MAX_VELOCITY / 3.5);
  waitForGoToPoint(long_goal, 2.0, 1000);
  hoodOpenDoor();

  delay(1500);

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