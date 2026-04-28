#include "driftless/robot/subsystems/hood/DirectHood.hpp"

namespace driftless::robot::subsystems::hood {
void DirectHood::init() { m_motors.init(); }

void DirectHood::run() {}

void DirectHood::setVoltage(double voltage) { m_motors.setVoltage(voltage); }

void DirectHood::open() {
  m_gate_pistons.extend();
  m_bump_pistons.extend();
  m_gate_open = true;
}

void DirectHood::close() {
  m_gate_pistons.retract();
  m_bump_pistons.retract();
  m_gate_open = false;
}

void DirectHood::toggleOpen() {
  m_gate_pistons.toggleState();
  m_bump_pistons.toggleState();
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
  m_upper_descore_pistons.extend();
  m_lower_descore_pistons.extend();
}

void DirectHood::retractDescore() {
  m_upper_descore_pistons.retract();
  m_lower_descore_pistons.retract();
}

void DirectHood::extendDescoreHalf() {
  m_lower_descore_pistons.extend();
  m_upper_descore_pistons.retract();
}

void DirectHood::bumpUp() { m_bump_pistons.extend(); }

void DirectHood::bumpDown() { m_bump_pistons.retract(); }

void DirectHood::toggleBump() { m_bump_pistons.toggleState(); }

void DirectHood::setCurrentLimit(double amps) {
  m_motors.setCurrentLimit(amps);
}

bool DirectHood::isOpen() { return m_gate_open; }

int DirectHood::getDescoreHeight() {
  int height{};
  height += static_cast<int>(m_lower_descore_pistons.isExtended());
  height += static_cast<int>(m_upper_descore_pistons.isExtended());

  return height;
}

void DirectHood::setMotors(hal::MotorGroup& motors) { m_motors = motors; }

void DirectHood::setHeightPistons(hal::PistonGroup& pistons) {
  m_height_pistons = pistons;
}

void DirectHood::setGatePistons(hal::PistonGroup& pistons) {
  m_gate_pistons = pistons;
}

void DirectHood::setLowerDescorePistons(hal::PistonGroup& pistons) {
  m_lower_descore_pistons = pistons;
}

void DirectHood::setUpperDescorePistons(hal::PistonGroup& pistons) {
  m_upper_descore_pistons = pistons;
}

void DirectHood::setBumpPistons(hal::PistonGroup& pistons) {
  m_bump_pistons = pistons;
}

bool DirectHood::isRaised() { return m_raised; }

bool DirectHood::isBumped() { return m_bump_pistons.isExtended(); }
}  // namespace driftless::robot::subsystems::hood