#include "driftless/control/motion/PIDHolonomicTurnBuilder.hpp"

namespace driftless::control::motion {
PIDHolonomicTurnBuilder* PIDHolonomicTurnBuilder::withDelayer(
    const std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return this;
}

PIDHolonomicTurnBuilder* PIDHolonomicTurnBuilder::withMutex(std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return this;
}

PIDHolonomicTurnBuilder* PIDHolonomicTurnBuilder::withTask(std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

PIDHolonomicTurnBuilder* PIDHolonomicTurnBuilder::withRotationalPID(PID rotational_pid) {
  m_rotational_pid = rotational_pid;
  return this;
}

PIDHolonomicTurnBuilder* PIDHolonomicTurnBuilder::withTargetTolerance(double target_tolerance) {
  m_target_tolerance = target_tolerance;
  return this;
}

PIDHolonomicTurnBuilder* PIDHolonomicTurnBuilder::withTargetVelocity(double target_velocity) {
  m_target_velocity = target_velocity;
  return this;
}

std::unique_ptr<PIDHolonomicTurn> PIDHolonomicTurnBuilder::build() {
  std::unique_ptr<PIDHolonomicTurn> pid_holonomic_turn{std::make_unique<PIDHolonomicTurn>()};

  pid_holonomic_turn->setDelayer(m_delayer);
  pid_holonomic_turn->setMutex(m_mutex);
  pid_holonomic_turn->setTask(m_task);
  pid_holonomic_turn->setRotationalPID(m_rotational_pid);
  pid_holonomic_turn->setTargetTolerance(m_target_tolerance);
  pid_holonomic_turn->setTargetVelocity(m_target_velocity);

  return pid_holonomic_turn;
}
}  // namespace driftless::control::motion