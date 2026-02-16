#include "driftless/control/trajectory/trajectory_follower/PIDTrajectoryFollower.hpp"

namespace driftless::control::trajectory::trajectory_follower {
void PIDTrajectoryFollower::taskLoop(void* params) {
  PIDTrajectoryFollower* follower = static_cast<PIDTrajectoryFollower*>(params);

  while (true) {
    follower->taskUpdate();
  }
}

void PIDTrajectoryFollower::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (!m_paused && !m_target_reached) {
    auto position = getRobotPosition();

    m_elapsed_time += m_clock->getTime() - m_last_time;

    m_last_time = m_clock->getTime();

    TrajectoryPoint current_point{m_trajectory[m_trajectory.size() - 1]};

    if (m_elapsed_time / 20 < m_trajectory.size()) {
      current_point = m_trajectory[m_elapsed_time / 20];
    }

    if (calculateDistanceToTarget(position) < m_target_tolerance &&
        std::sqrt(position.xV * position.xV + position.yV * position.yV) <
            m_target_velocity) {
      m_target_reached = true;
      setDriveMotionVector(0, 0, 0);
    } else {
      updateVelocity(position, current_point);
    }
  }

  if (m_mutex) {
    m_mutex->give();
  }

  m_delayer->delay(TASK_DELAY);
}

void PIDTrajectoryFollower::setDriveMotionVector(double x_velocity,
                                                 double y_velocity,
                                                 double angular_velocity) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
      robot::subsystems::ESubsystemCommand::
          HOLONOMIC_DRIVE_TRAIN_SET_MOTION_VECTOR,
      robot::subsystems::holonomic_drive_train::HolonomicMotionVector{
          x_velocity, y_velocity, angular_velocity});
}

driftless::robot::subsystems::odometry::Position
PIDTrajectoryFollower::getRobotPosition() {
  return *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
      robot::subsystems::ESubsystem::ODOMETRY,
      robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION));
}

double PIDTrajectoryFollower::calculateDistanceToTarget(
    const driftless::robot::subsystems::odometry::Position& position) {
  TrajectoryPoint target_point{m_trajectory.back()};

  double dx = target_point.m_x - position.x;
  double dy = target_point.m_y - position.y;

  return std::sqrt(dx * dx + dy * dy);
}

void PIDTrajectoryFollower::updateVelocity(
    const driftless::robot::subsystems::odometry::Position& position,
    const TrajectoryPoint& target_point) {
  double x_velocity = target_point.m_x_velocity;
  double y_velocity = target_point.m_y_velocity;
  double angular_velocity = target_point.m_angular_velocity;

  x_velocity += m_x_pid.getControlValue(position.x, target_point.m_x);
  y_velocity += m_y_pid.getControlValue(position.y, target_point.m_y);
  angular_velocity +=
      m_theta_pid.getControlValue(position.theta, target_point.m_heading);

  double out_x = x_velocity * std::sin(position.theta) -
                 y_velocity * std::cos(position.theta);
  double out_y = x_velocity * std::cos(position.theta) +
                 y_velocity * std::sin(position.theta);

  setDriveMotionVector(out_x, out_y, angular_velocity);
}

void PIDTrajectoryFollower::init() {
  m_x_pid.reset();
  m_y_pid.reset();
  m_theta_pid.reset();
}

void PIDTrajectoryFollower::run() {
  if (m_task) {
    m_task->start(taskLoop, this);
  }
}

void PIDTrajectoryFollower::pause() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = true;

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDTrajectoryFollower::resume() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = false;
  m_last_time = m_clock->getTime();

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDTrajectoryFollower::followTrajectory(
    const std::shared_ptr<robot::Robot>& robot,
    const std::vector<TrajectoryPoint>& trajectory) {
  if (m_mutex) {
    m_mutex->take();
  }
  m_robot = robot;
  m_trajectory = trajectory;
  m_paused = false;
  m_target_reached = false;
  m_elapsed_time = 0;
  m_last_time = m_clock->getTime();

  if (m_mutex) {
    m_mutex->give();
  }
}

bool PIDTrajectoryFollower::targetReached() { return m_target_reached; }

void PIDTrajectoryFollower::setDelayer(
    const std::unique_ptr<driftless::rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
}

void PIDTrajectoryFollower::setMutex(
    std::unique_ptr<driftless::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void PIDTrajectoryFollower::setTask(
    std::unique_ptr<driftless::rtos::ITask>& task) {
  m_task = std::move(task);
}

void PIDTrajectoryFollower::setClock(
    const std::unique_ptr<driftless::rtos::IClock>& clock) {
  m_clock = clock->clone();
}

void PIDTrajectoryFollower::setXPID(driftless::control::PID& x_pid) {
  m_x_pid = x_pid;
}

void PIDTrajectoryFollower::setYPID(driftless::control::PID& y_pid) {
  m_y_pid = y_pid;
}

void PIDTrajectoryFollower::setThetaPID(driftless::control::PID& theta_pid) {
  m_theta_pid = theta_pid;
}

void PIDTrajectoryFollower::setTargetTolerance(double target_tolerance) {
  m_target_tolerance = target_tolerance;
}

void PIDTrajectoryFollower::setTargetVelocity(double target_velocity) {
  m_target_velocity = target_velocity;
}
}  // namespace driftless::control::trajectory::trajectory_follower