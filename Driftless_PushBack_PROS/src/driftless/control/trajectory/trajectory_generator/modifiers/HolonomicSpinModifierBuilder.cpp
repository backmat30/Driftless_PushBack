#include "driftless/control/trajectory/trajectory_generator/modifiers/HolonomicSpinModifierBuilder.hpp"

namespace driftless::control::trajectory::trajectory_generator::modifiers {
HolonomicSpinModifierBuilder* HolonomicSpinModifierBuilder::withSpins(
    std::vector<SpinContext>& spins) {
  m_spins = spins;
  return this;
}

HolonomicSpinModifierBuilder* HolonomicSpinModifierBuilder::withMaxAcceleration(
    double max_acceleration) {
  m_max_acceleration = max_acceleration;
  return this;
}

HolonomicSpinModifierBuilder* HolonomicSpinModifierBuilder::withStartingHeading(
    double starting_heading) {
  m_starting_heading = starting_heading;
  return this;
}

std::unique_ptr<HolonomicSpinModifier> HolonomicSpinModifierBuilder::build() {
  std::unique_ptr<HolonomicSpinModifier> modifier{
      std::make_unique<HolonomicSpinModifier>()};
  modifier->setSpins(m_spins);
  modifier->setMaxAcceleration(m_max_acceleration);
  modifier->setStartingHeading(m_starting_heading);
  return modifier;
}
}