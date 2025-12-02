#include "driftless/control/trajectory/trajectory_generator/modifiers/HolonomicSpinModifier.hpp"

namespace driftless::control::trajectory::trajectory_generator::modifiers {
void HolonomicSpinModifier::applyModifier(
    std::vector<TrajectoryPoint>& trajectory) {
  int spin_index{};
  SpinContext current_spin{m_spins.at(spin_index)};

  for (int i{0}; i < current_spin.start_t / 0.02; ++i) {
    trajectory[i].m_heading = m_starting_heading;
    trajectory[i].m_angular_velocity = 0;
  }

  // apply each spin, and fill space between spins with constant heading using
  // the latest spin's end heading
  while (spin_index < m_spins.size()) {
    current_spin = m_spins.at(spin_index);
    double spin_duration = current_spin.end_t - current_spin.start_t;
    double initial_heading = (spin_index == 0)
                                 ? m_starting_heading
                                 : m_spins.at(spin_index - 1).end_angle;
    double total_angle_change =
        bindRadians(current_spin.end_angle - initial_heading);
    double direction = (total_angle_change > 0) ? 1 : -1;

    // Calculate the max angular velocity
    double max_angular_velocity{};
    double time_to_max_velocity{};
    if (std::pow(spin_duration, 2) <
        (4 * std::abs(total_angle_change) / m_max_acceleration)) {
      // Triangle profile
      time_to_max_velocity =
          std::sqrt(std::abs(total_angle_change) / m_max_acceleration);
      max_angular_velocity = m_max_acceleration * time_to_max_velocity;
    } else {
      max_angular_velocity =
          (-spin_duration +
           std::sqrt(std::pow(spin_duration, 2) -
                     (4 * std::abs(total_angle_change) / m_max_acceleration))) *
          (m_max_acceleration / -2);
      time_to_max_velocity = max_angular_velocity / m_max_acceleration;
    }

    int start_index = static_cast<int>(current_spin.start_t / 0.02);
    int end_index = static_cast<int>(current_spin.end_t / 0.02);
    int accel_end_index =
        start_index + static_cast<int>(time_to_max_velocity / 0.02);
    int decel_start_index =
        end_index - static_cast<int>(time_to_max_velocity / 0.02);

    // Acceleration phase
    for (int i = start_index; i < accel_end_index && i < trajectory.size();
         ++i) {
      double t = (i - start_index) * 0.02;
      trajectory[i].m_angular_velocity = direction * m_max_acceleration * t;
      trajectory[i].m_heading =
          initial_heading + direction * 0.5 * m_max_acceleration * t * t;
    }

    // Constant velocity phase
    for (int i = accel_end_index;
         i < decel_start_index && i < trajectory.size(); ++i) {
      double t = (i - accel_end_index) * 0.02;
      trajectory[i].m_angular_velocity = direction * max_angular_velocity;
      trajectory[i].m_heading =
          initial_heading +
          direction * (0.5 * m_max_acceleration * time_to_max_velocity *
                           time_to_max_velocity +
                       max_angular_velocity * t);
    }

    // Deceleration phase
    for (int i = decel_start_index; i < end_index && i < trajectory.size();
         ++i) {
      double t = (i - decel_start_index) * 0.02;
      trajectory[i].m_angular_velocity =
          direction * (max_angular_velocity - m_max_acceleration * t);
      trajectory[i].m_heading =
          initial_heading +
          direction *
              (0.5 * m_max_acceleration * time_to_max_velocity *
                   time_to_max_velocity +
               max_angular_velocity *
                   (spin_duration - 2 * time_to_max_velocity) +
               max_angular_velocity * t - 0.5 * m_max_acceleration * t * t);
    }
    // Ensure final heading is exact
    if (end_index < trajectory.size()) {
      trajectory[end_index].m_heading = current_spin.end_angle;
      trajectory[end_index].m_angular_velocity = 0;
    }

    // Fill in the gap between the end of this spin and the start of the next
    // spin
    int next_start_index =
        (spin_index + 1 < m_spins.size())
            ? static_cast<int>(m_spins.at(spin_index + 1).start_t / 0.02)
            : trajectory.size();
    for (int i = end_index + 1; i < next_start_index && i < trajectory.size();
         ++i) {
      trajectory[i].m_heading = current_spin.end_angle;
      trajectory[i].m_angular_velocity = 0;
    }
    spin_index++;
  }
}

void HolonomicSpinModifier::setSpins(std::vector<SpinContext>& spins) {
  m_spins = spins;
}

void HolonomicSpinModifier::setMaxAcceleration(double max_acceleration) {
  m_max_acceleration = max_acceleration;
}

void HolonomicSpinModifier::setStartingHeading(double starting_heading) {
  m_starting_heading = starting_heading;
}
}  // namespace driftless::control::trajectory::trajectory_generator::modifiers