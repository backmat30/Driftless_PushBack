#include "driftless/control/motion/PIDHolonomicGoToPointBuilder.hpp"

namespace driftless::control::motion {
PIDHolonomicGoToPointBuilder* PIDHolonomicGoToPointBuilder::withDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
  return this;
}

PIDHolonomicGoToPointBuilder* PIDHolonomicGoToPointBuilder::withMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return this;
}

PIDHolonomicGoToPointBuilder* PIDHolonomicGoToPointBuilder::withTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

PIDHolonomicGoToPointBuilder* PIDHolonomicGoToPointBuilder::withXPID(
    const PID& pid) {
  m_x_pid = pid;
  return this;
}

PIDHolonomicGoToPointBuilder* PIDHolonomicGoToPointBuilder::withYPID(
    const PID& pid) {
  m_y_pid = pid;
  return this;
}

PIDHolonomicGoToPointBuilder* PIDHolonomicGoToPointBuilder::withRotationalPID(
    const PID& pid) {
  m_rotational_pid = pid;
  return this;
}

PIDHolonomicGoToPointBuilder*
PIDHolonomicGoToPointBuilder::withDistanceTolerance(double tolerance) {
  m_distance_tolerance = tolerance;
  return this;
}

PIDHolonomicGoToPointBuilder*
PIDHolonomicGoToPointBuilder::withVelocityTolerance(double tolerance) {
  m_velocity_tolerance = tolerance;
  return this;
}

PIDHolonomicGoToPointBuilder*
PIDHolonomicGoToPointBuilder::withAngularTolerance(double tolerance) {
  m_angular_tolerance = tolerance;
  return this;
}

std::unique_ptr<PIDHolonomicGoToPoint> PIDHolonomicGoToPointBuilder::build() {
  std::unique_ptr<PIDHolonomicGoToPoint> go_to_point{
      std::make_unique<PIDHolonomicGoToPoint>()};

  go_to_point->setDelayer(m_delayer);
  go_to_point->setMutex(m_mutex);
  go_to_point->setXPID(m_x_pid);
  go_to_point->setYPID(m_y_pid);
  go_to_point->setRotationalPID(m_rotational_pid);
  go_to_point->setDistanceTolerance(m_distance_tolerance);
  go_to_point->setVelocityTolerance(m_velocity_tolerance);
  go_to_point->setAngularTolerance(m_angular_tolerance);

  return go_to_point;
}
}  // namespace driftless::control::motion