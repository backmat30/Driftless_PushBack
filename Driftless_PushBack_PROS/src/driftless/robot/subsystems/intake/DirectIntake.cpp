#include "driftless/robot/subsystems/intake/DirectIntake.hpp"

namespace driftless::robot::subsystems::intake {
  void DirectIntake::init() {
    m_motors.init();
  }

  void DirectIntake::run() {}

  void DirectIntake::setVoltage(double voltage) {
    m_motors.setVoltage(voltage);
  }

  void DirectIntake::deploy() {
    m_pistons.extend();
  }

  void DirectIntake::retract() {
    m_pistons.retract();
  }

  void DirectIntake::setMotors(hal::MotorGroup& motors) {
    m_motors = motors;
  }

  void DirectIntake::setPistons(hal::PistonGroup& pistons) {
    m_pistons = pistons;
  }
}