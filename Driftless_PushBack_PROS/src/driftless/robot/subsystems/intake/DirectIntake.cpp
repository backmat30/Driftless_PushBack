#include "driftless/robot/subsystems/intake/DirectIntake.hpp"

namespace driftless::robot::subsystems::intake {
  void DirectIntake::init() {
    m_front_motors.init();
    m_intermediary_motors.init();
    m_back_motors.init();
  }

  void DirectIntake::run() {}

  void DirectIntake::setFrontVoltage(double voltage) {
    m_front_motors.setVoltage(voltage);
  }

  void DirectIntake::setIntermediaryVoltage(double voltage) {
    m_intermediary_motors.setVoltage(voltage);
  }

  void DirectIntake::setBackVoltage(double voltage) {
    m_back_motors.setVoltage(voltage);
  }

  void DirectIntake::setVerticalVoltage(double voltage) {
    m_vertical_motors.setVoltage(voltage);
  }

  void DirectIntake::deploy() {
    m_back_pistons.extend();
  }

  void DirectIntake::retract() {
    m_back_pistons.retract();
  }

  bool DirectIntake::isDeployed() {
    return m_back_pistons.isExtended();
  }

  void DirectIntake::setFrontMotors(hal::MotorGroup& motors) {
    m_front_motors = motors;
  }

  void DirectIntake::setIntermediaryMotors(hal::MotorGroup& motors) {
    m_intermediary_motors = motors;
  }

  void DirectIntake::setBackMotors(hal::MotorGroup& motors) {
    m_back_motors = motors;
  }

  void DirectIntake::setBackPistons(hal::PistonGroup& pistons) {
    m_back_pistons = pistons;
  }

  void DirectIntake::setVerticalMotors(hal::MotorGroup& motors) {
    m_vertical_motors = motors;
  }
}