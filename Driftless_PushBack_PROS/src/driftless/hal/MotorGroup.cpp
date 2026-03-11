#include "driftless/hal/MotorGroup.hpp"

namespace driftless {
namespace hal {
void MotorGroup::addMotor(std::unique_ptr<io::IMotor>& motor) {
  motors.push_back(std::move(motor));
}

void MotorGroup::init() {
  for (auto& motor : motors)
    if (motor) motor->initialize();
}

double MotorGroup::getGearRatio() {
  double gear_ratio{};
  if (!motors.empty() && motors.front())
    gear_ratio = motors.front()->getGearRatio();
  return gear_ratio;
}

double MotorGroup::getAngularVelocity() {
  double average_velocity{};
  if (!motors.empty()) {
    for (auto& motor : motors)
      if (motor) average_velocity += motor->getAngularVelocity();
    average_velocity /= motors.size();
  }

  return average_velocity;
}

double MotorGroup::getPosition() {
  double average_position{};
  if (!motors.empty()) {
    for (auto& motor : motors)
      if (motor) average_position += motor->getPosition();
    average_position /= motors.size();
  }

  return average_position;
}

double MotorGroup::getEfficiency() {
  double average_efficiency{};
  if (!motors.empty()) {
    for (auto& motor : motors) {
      if (motor) {
        average_efficiency += motor->getEfficiency();
      }
    }
    average_efficiency / motors.size();
  }

  return average_efficiency;
}

void MotorGroup::setVoltage(double volts) {
  for (auto& motor : motors)
    if (motor) motor->setVoltage(volts);
}

void MotorGroup::setVelocity(double velocity) {
  for (auto& motor : motors)
    if (motor) motor->setVelocity(velocity);
}

void MotorGroup::setCurrentLimit(double amps) {
  for (auto& motor : motors) {
    if (motor) motor->setCurrentLimit(amps);
  }
}

void MotorGroup::setPosition(double position) {
  for (auto& motor : motors)
    if (motor) motor->setPosition(position);
}

MotorGroup& MotorGroup::operator=(MotorGroup& rhs) {
  motors.clear();
  for (uint8_t i{0}; i < rhs.motors.size(); ++i)
    motors.push_back(std::move(rhs.motors.at(i)));
  rhs.motors.clear();
  return *this;
}
}  // namespace hal
}  // namespace driftless