#include "driftless/io/motor_voltage_control/PartialFeedForwardVoltageController.hpp"

namespace driftless::io::motor_voltage_control {
PartialFeedForwardVoltageController::PartialFeedForwardVoltageController(
    double kS, double kV)
    : m_kS(kS), m_kV(kV) {}

double PartialFeedForwardVoltageController::getVoltage(double velocity,
                                                       double acceleration) {
  double voltage{};

  int direction{0};
  if (velocity > 0) {
    direction = 1;
  } else if (velocity < 0) {
    direction = -1;
  }

  voltage = m_kS * direction + m_kV * velocity;
  return voltage;
}
}