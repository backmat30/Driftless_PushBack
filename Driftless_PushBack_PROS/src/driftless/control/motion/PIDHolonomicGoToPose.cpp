#include "driftless/control/motion/PIDHolonomicGoToPose.hpp"

#include "pros/screen.hpp"
namespace driftless::control::motion {
void PIDHolonomicGoToPose::taskLoop(void* params) {
  PIDHolonomicGoToPose* go_to_point{static_cast<PIDHolonomicGoToPose*>(params)};

  while (true) {
    go_to_point->taskUpdate();
  }
}

void PIDHolonomicGoToPose::setDriveMotionVector(double x_velocity,
                                                double y_velocity,
                                                double angular_velocity) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
                       robot::subsystems::ESubsystemCommand::
                           HOLONOMIC_DRIVE_TRAIN_SET_MOTION_VECTOR,
                       x_velocity, y_velocity, angular_velocity);
}

robot::subsystems::odometry::Position PIDHolonomicGoToPose::getPosition() {
  return *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
      robot::subsystems::ESubsystem::ODOMETRY,
      robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION));
}

void PIDHolonomicGoToPose::updateVelocity(double x_distance, double y_distance,
                                          double current_heading,
                                          double angular_distance) {
  double x_velocity{m_x_pid.getControlValue(0, x_distance)};
  double y_velocity{m_y_pid.getControlValue(0, y_distance)};
  double angular_velocity{
      m_rotational_pid.getControlValue(0, angular_distance)};

  double velocity_scalar{1.0};
  double velocity{std::sqrt(x_velocity * x_velocity + y_velocity * y_velocity)};

  double true_max_velocity = std::min(
      m_max_velocity, std::sqrt(2 * m_max_linear_acceleration *
                                std::abs(std::sqrt(x_distance * x_distance +
                                                   y_distance * y_distance))));
  if (velocity > true_max_velocity) {
    velocity_scalar = true_max_velocity / velocity;
  }

  x_velocity *= velocity_scalar;
  y_velocity *= velocity_scalar;

  if (angular_velocity > m_max_rotational_velocity) {
    angular_velocity = m_max_rotational_velocity;
  }

  double out_x = x_velocity * std::sin(current_heading) -
                 y_velocity * std::cos(current_heading);
  double out_y = x_velocity * std::cos(current_heading) +
                 y_velocity * std::sin(current_heading);

  setDriveMotionVector(out_x, out_y, angular_velocity);
}

void PIDHolonomicGoToPose::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (!m_paused && !m_target_reached) {
    robot::subsystems::odometry::Position position{getPosition()};

    double x_distance{m_target_point.getX() - position.x};
    double y_distance{m_target_point.getY() - position.y};
    double angular_distance{
        bindRadians(m_target_point.getTheta() - position.theta)};

    double distance_to_target{
        std::sqrt(x_distance * x_distance + y_distance * y_distance)};
    double velocity{
        std::sqrt(position.xV * position.xV + position.yV * position.yV)};

    if (distance_to_target < m_distance_tolerance &&
        velocity < m_velocity_tolerance) {
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

void PIDHolonomicGoToPose::init() {
  m_x_pid.reset();
  m_y_pid.reset();
  m_rotational_pid.reset();
}

void PIDHolonomicGoToPose::run() {
  if (m_task) {
    m_task->start(&PIDHolonomicGoToPose::taskLoop, this);
  }
}

void PIDHolonomicGoToPose::pause() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = true;

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDHolonomicGoToPose::resume() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDHolonomicGoToPose::goToPose(const std::shared_ptr<robot::Robot>& robot,
                                    double velocity, double angular_velocity,
                                    double linear_acceleration, Point point) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_robot = robot;
  m_max_velocity = velocity;
  m_max_rotational_velocity = angular_velocity;
  m_max_linear_acceleration = linear_acceleration;
  m_target_point = point;
  robot::subsystems::odometry::Position start_pos = getPosition();
  m_initial_point = Point{start_pos.x, start_pos.y, start_pos.theta};
  m_target_reached = false;
  m_paused = false;

  m_x_pid.reset();
  m_y_pid.reset();
  m_rotational_pid.reset();

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDHolonomicGoToPose::setVelocity(double velocity) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_max_velocity = velocity;

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDHolonomicGoToPose::setAngularVelocity(double angular_velocity) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_max_rotational_velocity = angular_velocity;

  if (m_mutex) {
    m_mutex->give();
  }
}

bool PIDHolonomicGoToPose::targetReached() { return m_target_reached; }

void PIDHolonomicGoToPose::setDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
}

void PIDHolonomicGoToPose::setMutex(std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void PIDHolonomicGoToPose::setTask(std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
}

void PIDHolonomicGoToPose::setXPID(PID x_pid) { m_x_pid = x_pid; }

void PIDHolonomicGoToPose::setYPID(PID y_pid) { m_y_pid = y_pid; }

void PIDHolonomicGoToPose::setRotationalPID(PID rotational_pid) {
  m_rotational_pid = rotational_pid;
}

void PIDHolonomicGoToPose::setDistanceTolerance(double distance_tolerance) {
  m_distance_tolerance = distance_tolerance;
}

void PIDHolonomicGoToPose::setVelocityTolerance(double velocity_tolerance) {
  m_velocity_tolerance = velocity_tolerance;
}

void PIDHolonomicGoToPose::setAngularTolerance(double angular_tolerance) {
  m_angular_tolerance = angular_tolerance;
}
}  // namespace driftless::control::motion