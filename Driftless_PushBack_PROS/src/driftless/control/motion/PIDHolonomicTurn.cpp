#include "driftless/control/motion/PIDHolonomicTurn.hpp"

namespace driftless::control::motion {
void PIDHolonomicTurn::taskLoop(void* params) {
  PIDHolonomicTurn* pid_turn{static_cast<PIDHolonomicTurn*>(params)};

  while (true) {
    pid_turn->taskUpdate();
  }
}

void PIDHolonomicTurn::setDriveTurnVelocity(double velocity) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
                       robot::subsystems::ESubsystemCommand::
                           HOLONOMIC_DRIVE_TRAIN_SET_ANGULAR_VELOCITY,
                       velocity);
}

robot::subsystems::odometry::Position PIDHolonomicTurn::getPosition() {
  return *static_cast<driftless::robot::subsystems::odometry::Position*>(
      m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION));
}

double PIDHolonomicTurn::calculateAngleToTarget(
    driftless::robot::subsystems::odometry::Position position) {
  double x_error{m_target_point.getX() - position.x};
  double y_error{m_target_point.getY() - position.y};
  return std::atan2(y_error, x_error);
}

double PIDHolonomicTurn::calculateDriveVelocity(double current_angle,
                                                double target_angle) {
  double angle_error{bindRadians(target_angle - current_angle)};

  if (!m_forced_direction_reached) {
    // set the angle to negative if direction should be clockwise, otherwise
    // make the angle positive
    if (m_turn_direction == ETurnDirection::CLOCKWISE && angle_error > 0) {
      angle_error -= 2 * M_PI;
    } else if (m_turn_direction == ETurnDirection::COUNTERCLOCKWISE &&
               angle_error < 0) {
      angle_error += 2 * M_PI;
    }
    // if robot is within 1/4 of a rotation of the destination, confirm that the
    // forced direction has been achieved
    if (std::abs(angle_error) < M_PI / 2) {
      m_forced_direction_reached = true;
    }
  }

  double rotational_control{m_rotational_pid.getControlValue(0, angle_error)};

  if (std::abs(rotational_control) > m_max_velocity) {
    rotational_control *= m_max_velocity / std::abs(rotational_control);
  }
  return rotational_control;
}

void PIDHolonomicTurn::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (!m_target_reached && !m_paused) {
    driftless::robot::subsystems::odometry::Position position{getPosition()};

    double target_angle{calculateAngleToTarget(position)};
    double angle_error{bindRadians(target_angle - position.theta)};

    double drive_velocity{};
    if (std::abs(angle_error) < m_target_tolerance &&
        std::abs(position.thetaV) < m_target_velocity) {
      m_target_reached = true;
      drive_velocity = 0;
    } else {
      drive_velocity = calculateDriveVelocity(position.theta, target_angle);
    }
    setDriveTurnVelocity(drive_velocity);
  }

  if (m_mutex) {
    m_mutex->give();
  }

  if (m_delayer) {
    m_delayer->delay(TASK_DELAY);
  }
}

void PIDHolonomicTurn::init() { m_rotational_pid.reset(); }

void PIDHolonomicTurn::run() {
  m_task->start(&PIDHolonomicTurn::taskLoop, this);
}

void PIDHolonomicTurn::pause() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = true;
  setDriveTurnVelocity(0);

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDHolonomicTurn::resume() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = false;
  m_rotational_pid.reset();

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDHolonomicTurn::turnToAngle(
    const std::shared_ptr<driftless::robot::Robot>& robot, double velocity,
    double theta, ETurnDirection direction) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_robot = robot;
  driftless::robot::subsystems::odometry::Position position{getPosition()};

  m_turn_direction = direction;
  m_max_velocity = velocity;

  double target_x = position.x + (TURN_TO_ANGLE_DISTANCE * std::cos(theta));
  double target_y = position.y + (TURN_TO_ANGLE_DISTANCE * std::sin(theta));
  m_target_point.setX(target_x);
  m_target_point.setY(target_y);

  m_target_reached = false;
  m_forced_direction_reached = false;

  m_rotational_pid.reset();

  m_paused = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

void PIDHolonomicTurn::turnToPoint(
    const std::shared_ptr<driftless::robot::Robot>& robot, double velocity,
    Point point, ETurnDirection direction) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_robot = robot;
  m_turn_direction = direction;
  m_max_velocity = velocity;
  m_target_point = point;

  m_target_reached = false;
  m_forced_direction_reached = false;

  m_rotational_pid.reset();

  m_paused = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

bool PIDHolonomicTurn::targetReached() { return m_target_reached; }

void PIDHolonomicTurn::setDelayer(
    const std::unique_ptr<driftless::rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
}

void PIDHolonomicTurn::setMutex(
    std::unique_ptr<driftless::rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void PIDHolonomicTurn::setTask(std::unique_ptr<driftless::rtos::ITask>& task) {
  m_task = std::move(task);
}

void PIDHolonomicTurn::setRotationalPID(PID rotational_pid) {
  m_rotational_pid = rotational_pid;
}

void PIDHolonomicTurn::setTargetTolerance(double tolerance) {
  m_target_tolerance = tolerance;
}

void PIDHolonomicTurn::setTargetVelocity(double velocity) {
  m_target_velocity = velocity;
}
}  // namespace driftless::control::motion