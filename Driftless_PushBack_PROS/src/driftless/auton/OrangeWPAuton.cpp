#include "driftless/auton/OrangeWPAuton.hpp"

namespace driftless::auton {
OrangeWPAuton::OrangeWPAuton() : AAuton("O_WP") {}

void OrangeWPAuton::init(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
}

void OrangeWPAuton::run(
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
  setOdomPosition(91, 20.0, M_PI);
  startColorSort(m_alliance->getAlliance());

  // ROUTE GOES HERE
  intakeFront();

  // go to matchloader
  goToPose(matchload_lineup, MAX_VELOCITY, MAX_ANGULAR_VELOCITY);
  delay(750);
  setGoToPoseVelocity(MAX_VELOCITY / 1.5);
  waitForGoToPoint(matchload_lineup, 5.0, 1000);
  goToPose(matchload, MAX_VELOCITY / 1.5, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(matchload, 2.0, 1500);
  deployBackIntakeArms();
  intakeBackToHood();

  delay(450);
  hoodRaise();

  delay(200);
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

  delay(950);

  // go to descore
  goToPose(descore_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  middleDescore();
  delay(100);
  intakeStop();
  waitForGoToPose(descore_lineup, 1.0, 2000);

  goToPose(end_descore, MAX_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(end_descore, 2.5, 3000);

  // grab two blocks under goal
  deployDescore();
  goToPose(two_blocks_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(two_blocks_lineup, 1.5, 2000);

  hoodLower();
  hoodCloseDoor();
  retractDescore();
  intakeFront();

  goToPose(two_blocks_lineup_2, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(two_blocks_lineup_2, 1.5, 2000);

  goToPose(two_blocks, MAX_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(two_blocks, 1.0, 3000);

  // grab line of blocks
  goToPose(line_blocks_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(line_blocks_lineup, 1.5, 2500);

  goToPose(line_blocks_lineup_2, MAX_VELOCITY / 5.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(line_blocks_lineup_2, 1.0, 1000);

  goToPose(line_blocks, MAX_VELOCITY / 4.5, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(line_blocks, 1.0, 3500);

  // score in top middle goal
  hoodBumpUp();
  goToPose(top_goal_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(top_goal_lineup, 2.0, 3000);

  intakeStop();
  // hoodOpenDoor();

  goToPose(top_goal, MAX_VELOCITY / 2.5, MAX_ANGULAR_VELOCITY);
  waitForGoToPose(top_goal, 1.0, 1250);

  // intakeFront();
  // delay(2300);
  // hoodRaise();
  // delay(100);

  // // go back to matchload
  // goToPose(matchload, MAX_VELOCITY / 1.25, MAX_ANGULAR_VELOCITY);
  // waitForGoToPose(matchload, 7.0, 3000);
  // setGoToPoseVelocity(MAX_VELOCITY / 2.0);
  // waitForGoToPose(matchload, 1.0, 1000);

  // hoodCloseDoor();
  // hoodLower();
  // deployBackIntakeArms();
  // intakeBack();
  // delay(2000);
  // retractBackIntakeArms();

  // // go to bottom goal
  // goToPose(bottom_goal, MAX_VELOCITY / 1.5, MAX_ANGULAR_VELOCITY);
  // waitForGoToPoint(bottom_goal, 10.0, 2500);
  // setGoToPointVelocity(MAX_VELOCITY / 2.5);
  // waitForGoToPose(bottom_goal, 1.0, 1500);

  // outtakeFront(10);
  // delay(500);
  // hoodRaise();
  // delay(2150);
  // intakeStop();

  // goToPose(descore_2_lineup, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  // waitForGoToPose(descore_2_lineup, 1.0, 2000);

  // middleDescore();

  // goToPose(descore_2, MAX_VELOCITY / 2.0, MAX_ANGULAR_VELOCITY);
  // waitForGoToPose(descore_2, 1.0, 2000);

  // goToPose(descore_2_end, MAX_VELOCITY / 3.0, MAX_ANGULAR_VELOCITY);
  // waitForGoToPose(descore_2, 1.0, 2000);

  // leave at end
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 8, "Runtime: %7.2f",
                      (getTime() - start_time) / 1000.0);
}
}  // namespace driftless::auton