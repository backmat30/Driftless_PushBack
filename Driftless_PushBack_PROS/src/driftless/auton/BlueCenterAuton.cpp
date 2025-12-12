#include "driftless/auton/BlueCenterAuton.hpp"

namespace driftless::auton
{
  BlueCenterAuton::BlueCenterAuton() : AAuton("Blue_Center") {}

  void BlueCenterAuton::init(std::shared_ptr<driftless::robot::Robot> &robot,
                             std::shared_ptr<driftless::control::ControlSystem> &control_system)
  {
    m_robot = robot;
    m_control_system = control_system;
  }

  void BlueCenterAuton::run(std::shared_ptr<driftless::robot::Robot> &robot,
                            std::shared_ptr<driftless::control::ControlSystem> &control_system,
                            std::shared_ptr<driftless::alliance::IAlliance> &alliance,
                            std::shared_ptr<rtos::IClock> &clock,
                            std::unique_ptr<rtos::IDelayer> &delayer)
  {
    m_robot = robot;
    m_control_system = control_system;
    m_alliance = alliance;
    m_clock = clock;
    m_delayer = delayer->clone();

    uint32_t start_time{getTime()};
    setOdomPosition(56.0, 20.0, 0.0);
    startColorSort(m_alliance->getAlliance());
    intakeFront();

    //go to matchloader
    goToPoint(matchload_lineup, MAX_VELOCITY);
    delay(750);
    setGoToPointVelocity(MAX_VELOCITY / 2.0);
    delay(200);
    goToPoint(matchload, MAX_VELOCITY / 2.0);
    waitForGoToPoint(matchload, 2.0, 1500);
    deployBackIntakeArms();
    intakeBackToHood();

    delay(1250);
    outtakeFront();

    //score blocks from match loader
    goToPoint(long_goal_lineup, MAX_VELOCITY / 1.5);
    waitForGoToPoint(long_goal_lineup, 10.0, 1500);
    retractBackIntakeArms();
    waitForGoToPoint(long_goal_lineup, 8.0, 1500);
    goToPoint(long_goal_lineup_2, MAX_VELOCITY / 1.75);
    waitForGoToPoint(long_goal_lineup_2, 4.0, 1500);
    intakeFront();
    hoodRaise();
    goToPoint(long_goal, MAX_VELOCITY / 3.5);
    waitForGoToPoint(long_goal, 2.0, 2000);

    hoodOpenDoor();

    delay(1250);

    // go descore
    intakeStop();
    goToPoint(matchload_lineup, MAX_VELOCITY);
    delay(500);
    hoodCloseDoor();
    deployDescore();
    waitForGoToPoint(matchload_lineup, 6.0, 800);
    goToPoint(descore_lineup, MAX_VELOCITY);
    waitForGoToPoint(descore_lineup, 2.0, 2000);
    goToPoint(start_descore, MAX_VELOCITY / 2.0);
    waitForGoToPoint(start_descore, 2.0, 2000);
    retractDescore();
    goToPoint(end_descore, MAX_VELOCITY / 2.0);
    waitForGoToPoint(end_descore, 2.0, 2750);

    // go middle two blocks
    delay(100);
    deployDescore();
    delay(250);
    goToPoint(center_two_blocks_lineup, MAX_VELOCITY / 2.0);
    waitForGoToPoint(center_two_blocks_lineup, 2.0, 1000);
    hoodLower();
    retractDescore();
    intakeFront();
    delay(250);
    // goToPoint(center_two_blocks, MAX_VELOCITY / 3.0);
    // waitForGoToPoint(center_two_blocks, 2.0, 2000);
    goToPoint(center_two_blocks_2, MAX_VELOCITY / 3.0);
    waitForGoToPoint(center_two_blocks_2, 1.0, 2000);
    delay(100);
    goToPoint(center_two_blocks_3, MAX_VELOCITY / 3.0);
    waitForGoToPoint(center_two_blocks_3, 1.0, 2000);

    // go middle 6 blocks
    delay(100);
    goToPoint(center_six_blocks_lineup, MAX_VELOCITY / 2.0);
    waitForGoToPoint(center_six_blocks_lineup, 2.0, 2000);
    goToPoint(center_six_blocks, MAX_VELOCITY / 3.0);
    waitForGoToPoint(center_six_blocks, 1.0, 2000);
    // goToPoint(center_six_blocks_2, MAX_VELOCITY / 3.0);
    // waitForGoToPoint(center_six_blocks_2, 1, 2000);
    goToPoint(center_six_blocks_3, MAX_VELOCITY / 3.5);
    waitForGoToPoint(center_six_blocks_3, 1.5, 2000);

    // go bottom goal
    // goToPoint(bottom_goal_lineup, MAX_VELOCITY / 2.0);
    // waitForGoToPoint(bottom_goal_lineup, 1.0, 2000);
    //goToPoint(bottom_goal, MAX_VELOCITY / 2.0);
    //waitForGoToPoint(bottom_goal, 2.0, 2000);
    goToPoint(bottom_goal_1, MAX_VELOCITY);
    waitForGoToPoint(bottom_goal_1, 5.0, 1750);
    goToPoint(bottom_goal_2, MAX_VELOCITY / 2);
    waitForGoToPoint(bottom_goal_2, 1.0, 400);
    delay(100);

    hoodSetVoltage(-12.0);
    outtakeFront();

    delay(3000);

    // go matchload again
    intakeStop();
    goToPoint(matchload_lineup_2, MAX_VELOCITY);
    waitForGoToPoint(matchload_lineup_2, 4, 3500);
    goToPoint(matchload, MAX_VELOCITY / 2.0);
    waitForGoToPoint(matchload, 2.0, 1500);
    deployBackIntakeArms();
    intakeBack();

    delay(1000);
    outtakeFront();

    // go park
    goToPoint(park_lineup, MAX_VELOCITY);
    delay(200);
    retractBackIntakeArms();
    intakeStop();
    waitForGoToPoint(park_lineup, 2.0, 2000);
    goToPoint(park_lineup_2, MAX_VELOCITY);
    waitForGoToPoint(park_lineup_2, 2.0, 2000);
    goToPoint(park_zone, MAX_VELOCITY);
    delay(3000);


    stopMotion();


    // leave at end
    pros::screen::print(pros::E_TEXT_LARGE_CENTER, 8, "Runtime: %7.2f",
                      (getTime() - start_time) / 1000.0);
  }
}