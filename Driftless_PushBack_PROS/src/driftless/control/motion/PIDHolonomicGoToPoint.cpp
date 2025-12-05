#include "driftless/control/motion/PIDHolonomicGoToPoint.hpp"

#include "pros/screen.hpp"
namespace driftless::control::motion {
void PIDHolonomicGoToPoint::taskLoop(void* params) {
  PIDHolonomicGoToPoint* go_to_point{
      static_cast<PIDHolonomicGoToPoint*>(params)};

  while (true) {
    go_to_point->taskUpdate();
  }
}

void PIDHolonomicGoToPoint::setDriveMotionVector(double x_velocity,
                                                 double y_velocity,
                                                 double angular_velocity) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
      robot::subsystems::ESubsystemCommand::
          HOLONOMIC_DRIVE_TRAIN_SET_MOTION_VECTOR,
      robot::subsystems::holonomic_drive_train::HolonomicMotionVector{
          x_velocity, y_velocity, angular_velocity});
}

robot::subsystems::odometry::Position PIDHolonomicGoToPoint::getPosition() {
  return *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
      robot::subsystems::ESubsystem::ODOMETRY,
      robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION));
}

void PIDHolonomicGoToPoint::updateVelocity(double x_distance, double y_distance,
                                           double current_heading,
                                           double angular_distance) {
  double x_velocity{m_x_pid.getControlValue(0, x_distance)};
  double y_velocity{m_y_pid.getControlValue(0, y_distance)};
  double angular_velocity{
      m_rotational_pid.getControlValue(0, angular_distance)};

  double out_x = x_velocity * std::sin(current_heading) -
                 y_velocity * std::cos(current_heading);
  double out_y = x_velocity * std::cos(current_heading) +
                 y_velocity * std::sin(current_heading);

  setDriveMotionVector(out_x, out_y, angular_velocity);
}

void PIDHolonomicGoToPoint::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (!m_paused && !m_target_reached) {
    robot::subsystems::odometry::Position position{getPosition()};

    double x_distance{m_target_point.getX() - position.x};
    double y_distance{m_target_point.getY() - position.y};
    double distance_to_target{
        std::sqrt(x_distance * x_distance + y_distance * y_distance)};
    double angular_distance{
        bindRadians(m_target_point.getTheta() - position.theta)};
    double velocity{
        std::sqrt(position.xV * position.xV + position.yV * position.yV)};

    if (distance_to_target < m_distance_tolerance &&
        velocity < m_velocity_tolerance &&
        std::abs(angular_distance) < m_angular_tolerance) {
      m_target_reached = true;
      setDriveMotionVector(0, 0, 0);
      pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 8, "Target Reached");
    } else {
      updateVelocity(x_distance, y_distance, position.theta, angular_distance);
    }
  }

  if (m_mutex) {
    m_mutex->give();
  }

  if (m_delayer) {
    m_delayer->delay(TASK_DELAY);
  }
}

void PIDHolonomicGoToPoint::init() {
  m_x_pid.reset();
  m_y_pid.reset();
  m_rotational_pid.reset();
}

void PIDHolonomicGoToPoint::run() {
  if (m_task) {
    m_task->start(&PIDHolonomicGoToPoint::taskLoop, this);
  }
}

void PIDHolonomicGoToPoint::pause() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = true;

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDHolonomicGoToPoint::resume() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDHolonomicGoToPoint::goToPoint(
    const std::shared_ptr<robot::Robot>& robot, double velocity, Point point) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_robot = robot;
  m_max_velocity = velocity;
  m_target_point = point;
  m_target_reached = false;
  m_paused = false;

  m_x_pid.reset();
  m_y_pid.reset();
  m_rotational_pid.reset();

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDHolonomicGoToPoint::setVelocity(double velocity) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_max_velocity = velocity;

  if (m_mutex) {
    m_mutex->give();
  }
}

bool PIDHolonomicGoToPoint::targetReached() { return m_target_reached; }

void PIDHolonomicGoToPoint::setDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
}

void PIDHolonomicGoToPoint::setMutex(std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void PIDHolonomicGoToPoint::setTask(std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
}

void PIDHolonomicGoToPoint::setXPID(PID x_pid) { m_x_pid = x_pid; }

void PIDHolonomicGoToPoint::setYPID(PID y_pid) { m_y_pid = y_pid; }

void PIDHolonomicGoToPoint::setRotationalPID(PID rotational_pid) {
  m_rotational_pid = rotational_pid;
}

void PIDHolonomicGoToPoint::setDistanceTolerance(double distance_tolerance) {
  m_distance_tolerance = distance_tolerance;
}

void PIDHolonomicGoToPoint::setVelocityTolerance(double velocity_tolerance) {
  m_velocity_tolerance = velocity_tolerance;
}

void PIDHolonomicGoToPoint::setAngularTolerance(double angular_tolerance) {
  m_angular_tolerance = angular_tolerance;
}
}  // namespace driftless::control::motion