#include "driftless/auton/FeedForwardTunerAuton.hpp"

namespace driftless::auton {
FeedForwardTunerAuton::FeedForwardTunerAuton() : AAuton("FF_TUNER") {}

void FeedForwardTunerAuton::init(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
}

void FeedForwardTunerAuton::run(
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

  double voltage = 3.0;
  double* wheel_speeds = static_cast<double*>(
      m_robot->getState(robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
                        robot::subsystems::ESubsystemState::
                            HOLONOMIC_DRIVE_TRAIN_GET_WHEEL_SPEED));
  do {
    for (int i = 0; i < 4; ++i) {
      m_robot->sendCommand(robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
                           robot::subsystems::ESubsystemCommand::
                               HOLONOMIC_DRIVE_TRAIN_SET_MODULE_VOLTAGE,
                           i, voltage);
      delayer->delay(50);
      wheel_speeds = static_cast<double*>(m_robot->getState(
          robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
          robot::subsystems::ESubsystemState::
              HOLONOMIC_DRIVE_TRAIN_GET_WHEEL_SPEED));
    }
    voltage -= 0.05;
  } while (wheel_speeds[0] != 0.0 && voltage > 0.0);

  do {
    for (int i = 0; i < 4; ++i) {
      m_robot->sendCommand(robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
                           robot::subsystems::ESubsystemCommand::
                               HOLONOMIC_DRIVE_TRAIN_SET_MODULE_VOLTAGE,
                           i, voltage);
      delayer->delay(50);
      wheel_speeds[i] = (static_cast<double*>(m_robot->getState(
          robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
          robot::subsystems::ESubsystemState::
              HOLONOMIC_DRIVE_TRAIN_GET_WHEEL_SPEED)))[i];
    }
    voltage += 0.01;
  } while (std::abs(wheel_speeds[0]) < 5.0);

  double base_voltage = voltage;
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 5, "Base Voltage: %.2f V",
                      base_voltage);

  for (int i = 0; i < 4; ++i) {
    stopMotion();
    wheel_speeds = static_cast<double*>(
        m_robot->getState(robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
                          robot::subsystems::ESubsystemState::
                              HOLONOMIC_DRIVE_TRAIN_GET_WHEEL_SPEED));

    voltage = 3.0;

    do {
      m_robot->sendCommand(robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
                           robot::subsystems::ESubsystemCommand::
                               HOLONOMIC_DRIVE_TRAIN_SET_MODULE_VOLTAGE,
                           i, voltage);
      delayer->delay(50);
      wheel_speeds = static_cast<double*>(m_robot->getState(
          robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
          robot::subsystems::ESubsystemState::
              HOLONOMIC_DRIVE_TRAIN_GET_WHEEL_SPEED));
      voltage -= 0.05;
    } while (wheel_speeds[i] != 0.0 && voltage > 0.0);

    do {
      m_robot->sendCommand(robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
                           robot::subsystems::ESubsystemCommand::
                               HOLONOMIC_DRIVE_TRAIN_SET_MODULE_VOLTAGE,
                           i, voltage);
      m_robot->sendCommand(robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
                           robot::subsystems::ESubsystemCommand::
                               HOLONOMIC_DRIVE_TRAIN_SET_MODULE_VOLTAGE,
                           3 - i, base_voltage);
      delayer->delay(50);
      wheel_speeds[i] = (static_cast<double*>(m_robot->getState(
          robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
          robot::subsystems::ESubsystemState::
              HOLONOMIC_DRIVE_TRAIN_GET_WHEEL_SPEED)))[i];
      voltage += 0.01;
    } while (std::abs(wheel_speeds[i]) < 1.0);

    stopMotion();
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, i + 6, "%d: %7.2f kS", i,
                        voltage);

    delay(1500);
  }
}
}  // namespace driftless::auton