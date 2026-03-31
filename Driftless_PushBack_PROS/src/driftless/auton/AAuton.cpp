#include "driftless/auton/AAuton.hpp"

namespace driftless::auton {
uint32_t AAuton::getTime() {
  uint32_t time{};
  if (m_clock) {
    time = m_clock->getTime();
  }

  return time;
}

void AAuton::delay(uint32_t delay_time) {
  if (m_delayer) {
    m_delayer->delay(delay_time);
  }
}

void AAuton::delayUntil(uint32_t time) {
  double current_time{};

  if (m_clock) {
    current_time = m_clock->getTime();
  }

  if (m_delayer && current_time < time) {
    m_delayer->delayUntil(time);
  }
}

robot::subsystems::odometry::Position AAuton::getOdomPosition() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};

  return position;
}

void AAuton::setOdomPosition(double x, double y, double theta) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ODOMETRY,
      robot::subsystems::ESubsystemCommand::ODOMETRY_SET_POSITION, x, y, theta);
}

void AAuton::followTrajectory(
    std::vector<control::trajectory::TrajectoryPoint>& trajectory) {
  m_control_system->sendCommand(control::EControl::TRAJECTORY_FOLLOWER,
                                control::EControlCommand::FOLLOW_TRAJECTORY,
                                m_robot, trajectory);
}

bool AAuton::trajectoryTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::TRAJECTORY_FOLLOWER,
      control::EControlState::TRAJECTORY_FOLLOWER_TARGET_REACHED))};

  return target_reached;
}

void AAuton::waitForTrajectory(control::trajectory::TrajectoryPoint& endpoint,
                               double tolerance, uint32_t timeout) {
  uint32_t start_time{getTime()};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  double distance_to_target{distance(current_position.x, current_position.y,
                                     endpoint.m_x, endpoint.m_y)};

  while (getTime() < start_time + timeout && !trajectoryTargetReached() &&
         std::abs(distance_to_target) > tolerance) {
    current_position = getOdomPosition();
    distance_to_target = distance(current_position.x, current_position.y,
                                  endpoint.m_x, endpoint.m_y);
    delay(LOOP_DELAY);
  }
}

void AAuton::goToPoint(control::Point target_point, double target_velocity) {
  m_control_system->sendCommand(
      control::EControl::MOTION, control::EControlCommand::GO_TO_POINT, m_robot,
      target_velocity, target_point.getX(), target_point.getY());
}

bool AAuton::goToPointTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION,
      control::EControlState::GO_TO_POINT_TARGET_REACHED))};

  return target_reached;
}

void AAuton::waitForGoToPoint(control::Point target_point, double tolerance,
                              uint32_t timeout) {
  uint32_t start_time{getTime()};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  double distance_to_target{distance(current_position.x, current_position.y,
                                     target_point.getX(), target_point.getY())};

  while (getTime() < start_time + timeout && !goToPointTargetReached() &&
         std::abs(distance_to_target) > tolerance) {
    current_position = getOdomPosition();
    distance_to_target = distance(current_position.x, current_position.y,
                                  target_point.getX(), target_point.getY());
    delay(LOOP_DELAY);
  }
}

void AAuton::setGoToPointVelocity(double velocity) {
  m_control_system->sendCommand(
      control::EControl::MOTION,
      control::EControlCommand::GO_TO_POINT_SET_VELOCITY, velocity);
}

void AAuton::goToPose(control::Point target_point, double target_velocity,
                      double target_angular_velocity,
                      double linear_acceleration) {
  m_control_system->sendCommand(
      control::EControl::MOTION, control::EControlCommand::GO_TO_POSE, m_robot,
      target_velocity, target_angular_velocity, linear_acceleration,
      target_point.getX(), target_point.getY(), target_point.getTheta());
}

bool AAuton::goToPoseTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION,
      control::EControlState::GO_TO_POSE_TARGET_REACHED))};

  return target_reached;
}

void AAuton::waitForGoToPose(control::Point target_point,
                             double position_tolerance, uint32_t timeout) {
  uint32_t start_time{getTime()};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  double distance_to_target{distance(current_position.x, current_position.y,
                                     target_point.getX(), target_point.getY())};

  while (getTime() < start_time + timeout && !goToPoseTargetReached() &&
         std::abs(distance_to_target) > position_tolerance) {
    current_position = getOdomPosition();
    distance_to_target = distance(current_position.x, current_position.y,
                                  target_point.getX(), target_point.getY());
    delay(LOOP_DELAY);
  }
}

void AAuton::setGoToPoseVelocity(double velocity) {
  m_control_system->sendCommand(
      control::EControl::MOTION,
      control::EControlCommand::GO_TO_POSE_SET_VELOCITY, velocity);
}

void AAuton::setGoToPoseAngularVelocity(double angular_velocity) {
  m_control_system->sendCommand(
      control::EControl::MOTION,
      control::EControlCommand::GO_TO_POSE_SET_ANGULAR_VELOCITY,
      angular_velocity);
}

void AAuton::turnToPoint(control::Point target_point, double target_velocity,
                         control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::TURN_TO_POINT,
                                m_robot, target_velocity, target_point.getX(),
                                target_point.getY(), direction);
}

void AAuton::turnToHeading(double heading, double target_velocity,
                           control::motion::ETurnDirection direction) {
  m_control_system->sendCommand(control::EControl::MOTION,
                                control::EControlCommand::TURN_TO_ANGLE,
                                m_robot, target_velocity, heading, direction);
}

bool AAuton::turnTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::MOTION, control::EControlState::TURN_TARGET_REACHED))};

  return target_reached;
}

void AAuton::waitForTurnToPoint(control::Point target_point, double tolerance,
                                uint32_t timeout) {
  uint32_t current_time{m_clock->getTime()};
  uint32_t end_time{current_time + timeout};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  double angle_difference{
      bindRadians(angle(current_position.x, current_position.y,
                        target_point.getX(), target_point.getY()) -
                  current_position.theta)};
  while (!turnTargetReached() && current_time < end_time &&
         std::abs(angle_difference) > tolerance) {
    current_time = m_clock->getTime();
    current_position = getOdomPosition();
    angle_difference =
        bindRadians(angle(current_position.x, current_position.y,
                          target_point.getX(), target_point.getY()) -
                    current_position.theta);
    m_delayer->delay(LOOP_DELAY);
  }
}

void AAuton::waitForTurnToAngle(double heading, double tolerance,
                                uint32_t timeout) {
  uint32_t current_time{m_clock->getTime()};
  uint32_t end_time{current_time + timeout};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  while (!turnTargetReached() && current_time < end_time &&
         std::abs(bindRadians(current_position.theta - heading)) > tolerance) {
    current_time = m_clock->getTime();
    current_position = getOdomPosition();
    m_delayer->delay(LOOP_DELAY);
  }
}

void AAuton::stopMotion() {
  m_control_system->pause();

  m_robot->sendCommand(
      robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
      robot::subsystems::ESubsystemCommand::
          HOLONOMIC_DRIVE_TRAIN_SET_MOTION_VECTOR,
      robot::subsystems::holonomic_drive_train::HolonomicMotionVector{0.0, 0.0,
                                                                      0.0});
}

void AAuton::intakeFront(double voltage) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_FRONT,
                       std::abs(voltage));
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                       robot::subsystems::ESubsystemCommand::HOOD_SET_VOLTAGE,
                       std::abs(voltage));

  bool is_hood_open{*static_cast<bool*>(
      m_robot->getState(robot::subsystems::ESubsystem::HOOD,
                        robot::subsystems::ESubsystemState::HOOD_IS_OPEN))};

  if (!is_hood_open) {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_SET_CURRENT_LIMIT, 1.35);
  } else {
    m_robot->sendCommand(
        robot::subsystems::ESubsystem::HOOD,
        robot::subsystems::ESubsystemCommand::HOOD_SET_CURRENT_LIMIT, 2.5);
  }
}

void AAuton::outtakeFront(double voltage) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_FRONT,
                       -std::abs(voltage));
}

void AAuton::intakeBack() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::INTAKE,
                       robot::subsystems::ESubsystemCommand::INTAKE_BACK_IN);

  m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                       robot::subsystems::ESubsystemCommand::HOOD_SET_VOLTAGE,
                       12.0);
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::HOOD,
      robot::subsystems::ESubsystemCommand::HOOD_SET_CURRENT_LIMIT, 1.35);
}

void AAuton::intakeBackToHood() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::INTAKE,
      robot::subsystems::ESubsystemCommand::INTAKE_BACK_TO_HOOD);

  m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                       robot::subsystems::ESubsystemCommand::HOOD_SET_VOLTAGE,
                       12.0);
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::HOOD,
      robot::subsystems::ESubsystemCommand::HOOD_SET_CURRENT_LIMIT, 1.35);
}

void AAuton::intakeStop() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::INTAKE,
      robot::subsystems::ESubsystemCommand::INTAKE_STOP_MOTION);
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                       robot::subsystems::ESubsystemCommand::HOOD_SET_VOLTAGE,
                       0.0);
}

void AAuton::deployBackIntakeArms() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::INTAKE,
      robot::subsystems::ESubsystemCommand::INTAKE_DEPLOY_ARMS);
}

void AAuton::retractBackIntakeArms() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::INTAKE,
      robot::subsystems::ESubsystemCommand::INTAKE_RETRACT_ARMS);
}

void AAuton::startColorSort(alliance::EAlliance alliance) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::INTAKE,
      robot::subsystems::ESubsystemCommand::INTAKE_START_COLOR_SORT, alliance);
}

void AAuton::pauseColorSort() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::INTAKE,
      robot::subsystems::ESubsystemCommand::INTAKE_PAUSE_COLOR_SORT);
}

void AAuton::hoodRaise() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                       robot::subsystems::ESubsystemCommand::HOOD_RAISE);
}

void AAuton::hoodBumpUp() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                       robot::subsystems::ESubsystemCommand::HOOD_BUMP_UP);
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                       robot::subsystems::ESubsystemCommand::HOOD_LOWER);
}

void AAuton::hoodLower() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                       robot::subsystems::ESubsystemCommand::HOOD_BUMP_DOWN);
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                       robot::subsystems::ESubsystemCommand::HOOD_LOWER);
}

void AAuton::hoodOpenDoor() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                       robot::subsystems::ESubsystemCommand::HOOD_OPEN_GATE);
}

void AAuton::hoodCloseDoor() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                       robot::subsystems::ESubsystemCommand::HOOD_CLOSE_GATE);
}

void AAuton::hoodSetVoltage(double voltage) {
  m_robot->sendCommand(robot::subsystems::ESubsystem::HOOD,
                       robot::subsystems::ESubsystemCommand::HOOD_SET_VOLTAGE,
                       voltage);
}

void AAuton::deployRake() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::RAKE,
                       robot::subsystems::ESubsystemCommand::RAKE_DEPLOY);
}

void AAuton::retractRake() {
  m_robot->sendCommand(robot::subsystems::ESubsystem::RAKE,
                       robot::subsystems::ESubsystemCommand::RAKE_RETRACT);
}

void AAuton::deployDescore() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::HOOD,
      robot::subsystems::ESubsystemCommand::HOOD_EXTEND_DESCORE);
}

void AAuton::middleDescore() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::HOOD,
      robot::subsystems::ESubsystemCommand::HOOD_EXTEND_DESCORE_HALF);
}

void AAuton::retractDescore() {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::HOOD,
      robot::subsystems::ESubsystemCommand::HOOD_RETRACT_DESCORE);
}

AAuton::AAuton(std::string name) : m_name{name} {}

std::string AAuton::getName() { return m_name; }
}  // namespace driftless::auton