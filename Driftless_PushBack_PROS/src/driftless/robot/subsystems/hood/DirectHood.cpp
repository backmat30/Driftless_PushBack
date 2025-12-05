#include "driftless/robot/subsystems/hood/DirectHood.hpp"

namespace driftless::robot::subsystems::hood {
void DirectHood::init() { m_motors.init(); }

void DirectHood::run() {}

void DirectHood::setVoltage(double voltage) {
  m_motors.setVoltage(voltage);
}

void DirectHood::open() {
  m_gate_pistons.extend();
  m_gate_open = true;
}

void DirectHood::close() {
  m_gate_pistons.retract();
  m_gate_open = false;
}

void DirectHood::toggleOpen() {
  m_gate_pistons.toggleState();
  m_gate_open = !m_gate_open;
}

void DirectHood::raise() {
  m_height_pistons.extend();
  m_raised = true;
}

void DirectHood::lower() {
  m_height_pistons.retract();
  m_raised = false;
}

void DirectHood::toggleRaised() {
  m_height_pistons.toggleState();
  m_raised = !m_raised;
}

void DirectHood::extendDescore() {
  m_descore_pistons.extend();
}

void DirectHood::retractDescore() {
  m_descore_pistons.retract();
}

void DirectHood::toggleDescore() {
  m_descore_pistons.toggleState();
}

void DirectHood::bumpUp() {
  m_bump_pistons.extend();
}

void DirectHood::bumpDown() {
  m_bump_pistons.retract();
}

void DirectHood::toggleBump() {
  m_bump_pistons.toggleState();
}

bool DirectHood::isOpen() {
  return m_gate_open;
}

void DirectHood::setMotors(hal::MotorGroup& motors) {
  m_motors = motors;
}

void DirectHood::setHeightPistons(hal::PistonGroup& pistons) {
  m_height_pistons = pistons;
}

void DirectHood::setGatePistons(hal::PistonGroup& pistons) {
  m_gate_pistons = pistons;
}

void DirectHood::setDescorePistons(hal::PistonGroup& pistons) {
  m_descore_pistons = pistons;
}

void DirectHood::setBumpPistons(hal::PistonGroup& pistons) {
  m_bump_pistons = pistons;
}

bool DirectHood::isRaised() {
  return m_raised;
}

bool DirectHood::isBumped() {
  return m_bump_pistons.isExtended();
}
}  // namespace driftless::robot::subsystems::hood