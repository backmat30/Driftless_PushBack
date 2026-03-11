#include "driftless/pros_adapters/ProsV5Motor.hpp"
namespace driftless {
namespace pros_adapters {
ProsV5Motor::ProsV5Motor(std::unique_ptr<pros::Motor>& motor)
    : m_motor{std::move(motor)} {}

ProsV5Motor::ProsV5Motor(
    std::unique_ptr<pros::Motor>& motor,
    std::unique_ptr<io::motor_voltage_control::IVoltageController>&
        voltage_controller)
    : m_motor{std::move(motor)},
      voltage_controller{std::move(voltage_controller)} {}

void ProsV5Motor::initialize() {
  if (m_motor) {
    m_motor->move_voltage(0);
    m_motor->tare_position();
    m_motor->set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    m_motor->set_encoder_units(
        pros::motor_encoder_units_e_t::E_MOTOR_ENCODER_ROTATIONS);
  }
}

double ProsV5Motor::getGearRatio() {
  double ratio{};

  if (m_motor) {
    pros::MotorGears gearing{m_motor->get_gearing()};
    if (cartridge_map.contains(gearing))
      ratio = cartridge_map.at(gearing);
    else
      ratio = NO_CARTRIDGE;
  }

  return ratio;
}

double ProsV5Motor::getAngularVelocity() {
  double angular_velocity{};

  if (m_motor)
    angular_velocity = m_motor->get_actual_velocity() * VELOCITY_CONVERSION;

  return angular_velocity;
}

double ProsV5Motor::getPosition() {
  double position{position_offset};

  if (m_motor) position += m_motor->get_position() * POSITION_CONVERSION;

  return position;
}

double ProsV5Motor::getEfficiency() {
  double efficiency{0.0};

  if (m_motor) {
    efficiency = m_motor->get_efficiency();
  }
  return efficiency;
}

void ProsV5Motor::setVoltage(double volts) {
  int millivolts{static_cast<int>(volts * VOLTAGE_CONVERSION)};
  millivolts = std::min(millivolts, MAX_MILLIVOLTS);
  millivolts = std::max(millivolts, -MAX_MILLIVOLTS);

  if (m_motor) m_motor->move_voltage(millivolts);
}

void ProsV5Motor::setVelocity(double velocity) {
  if (m_motor) {
    if (voltage_controller) {
      double volts{voltage_controller->getVoltage(
          velocity, velocity - getAngularVelocity())};
      setVoltage(volts);
    } else {
      double rpm{velocity / VELOCITY_CONVERSION};
      m_motor->move_velocity(rpm);
    }
  }
}

void ProsV5Motor::setCurrentLimit(double amps) {
  int milliamps{static_cast<int>(amps * CURRENT_CONVERSION)};
  milliamps = std::max(milliamps, 0);
  milliamps = std::min(milliamps, MAX_MILLIAMPS);

  if (m_motor) {
    m_motor->set_current_limit(milliamps);
  }
}

void ProsV5Motor::setPosition(double position) {
  position_offset = position;
  if (m_motor) m_motor->tare_position();
}
}  // namespace pros_adapters
}  // namespace driftless