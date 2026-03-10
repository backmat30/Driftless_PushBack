#include "driftless/control/motion/PIDHolonomicGoToPoseBuilder.hpp"

namespace driftless::control::motion {
PIDHolonomicGoToPoseBuilder* PIDHolonomicGoToPoseBuilder::withDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
  return this;
}

PIDHolonomicGoToPoseBuilder* PIDHolonomicGoToPoseBuilder::withMutex(
    std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
  return this;
}

PIDHolonomicGoToPoseBuilder* PIDHolonomicGoToPoseBuilder::withTask(
    std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
  return this;
}

PIDHolonomicGoToPoseBuilder* PIDHolonomicGoToPoseBuilder::withXPID(
    const PID& pid) {
  m_x_pid = pid;
  return this;
}

PIDHolonomicGoToPoseBuilder* PIDHolonomicGoToPoseBuilder::withYPID(
    const PID& pid) {
  m_y_pid = pid;
  return this;
}

PIDHolonomicGoToPoseBuilder* PIDHolonomicGoToPoseBuilder::withRotationalPID(
    const PID& pid) {
  m_rotational_pid = pid;
  return this;
}

PIDHolonomicGoToPoseBuilder* PIDHolonomicGoToPoseBuilder::withDistanceTolerance(
    double tolerance) {
  m_distance_tolerance = tolerance;
  return this;
}

PIDHolonomicGoToPoseBuilder* PIDHolonomicGoToPoseBuilder::withVelocityTolerance(
    double tolerance) {
  m_velocity_tolerance = tolerance;
  return this;
}

PIDHolonomicGoToPoseBuilder* PIDHolonomicGoToPoseBuilder::withAngularTolerance(
    double tolerance) {
  m_angular_tolerance = tolerance;
  return this;
}

std::unique_ptr<PIDHolonomicGoToPose> PIDHolonomicGoToPoseBuilder::build() {
  std::unique_ptr<PIDHolonomicGoToPose> go_to_pose{
      std::make_unique<PIDHolonomicGoToPose>()};

  go_to_pose->setDelayer(m_delayer);
  go_to_pose->setMutex(m_mutex);
  go_to_pose->setTask(m_task);
  go_to_pose->setXPID(m_x_pid);
  go_to_pose->setYPID(m_y_pid);
  go_to_pose->setRotationalPID(m_rotational_pid);
  go_to_pose->setDistanceTolerance(m_distance_tolerance);
  go_to_pose->setVelocityTolerance(m_velocity_tolerance);
  go_to_pose->setAngularTolerance(m_angular_tolerance);

  return go_to_pose;
}
}  // namespace driftless::control::motion