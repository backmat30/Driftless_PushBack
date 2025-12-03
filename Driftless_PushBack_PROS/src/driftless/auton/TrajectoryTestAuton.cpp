#include "driftless/auton/TrajectoryTestAuton.hpp"

#include <iostream>

namespace driftless::auton {
uint32_t TrajectoryTestAuton::getTime() { return m_clock->getTime(); }

robot::subsystems::odometry::Position TrajectoryTestAuton::getOdomPosition() {
  robot::subsystems::odometry::Position position{
      *static_cast<robot::subsystems::odometry::Position*>(m_robot->getState(
          robot::subsystems::ESubsystem::ODOMETRY,
          robot::subsystems::ESubsystemState::ODOMETRY_GET_POSITION))};

  return position;
}

void TrajectoryTestAuton::setOdomPosition(double x, double y, double theta) {
  m_robot->sendCommand(
      robot::subsystems::ESubsystem::ODOMETRY,
      robot::subsystems::ESubsystemCommand::ODOMETRY_SET_POSITION, x, y, theta);
}

void TrajectoryTestAuton::followTrajectory(
    std::vector<control::trajectory::TrajectoryPoint>& trajectory) {
  m_control_system->sendCommand(control::EControl::TRAJECTORY_FOLLOWER,
                                control::EControlCommand::FOLLOW_TRAJECTORY,
                                m_robot, trajectory);
}

bool TrajectoryTestAuton::trajectoryTargetReached() {
  bool target_reached{*static_cast<bool*>(m_control_system->getState(
      control::EControl::TRAJECTORY_FOLLOWER,
      control::EControlState::TRAJECTORY_FOLLOWER_TARGET_REACHED))};

  return target_reached;
}

void TrajectoryTestAuton::waitForTrajectory(
    control::trajectory::TrajectoryPoint& endpoint, double tolerance,
    uint32_t timeout) {
  uint32_t start_time{getTime()};
  robot::subsystems::odometry::Position current_position{getOdomPosition()};
  double distance_to_target{distance(current_position.x, current_position.y,
                                     endpoint.m_x, endpoint.m_y)};

  while (getTime() < start_time + timeout && !trajectoryTargetReached() &&
         std::abs(distance_to_target) > tolerance) {
    current_position = getOdomPosition();
    distance_to_target = distance(current_position.x, current_position.y,
                                  endpoint.m_x, endpoint.m_y);
    m_delayer->delay(LOOP_DELAY);
  }
}

void TrajectoryTestAuton::stopMotion() {
  m_control_system->sendCommand(
      control::EControl::TRAJECTORY_FOLLOWER,
      control::EControlCommand::FOLLOW_TRAJECTORY, m_robot,
      std::vector<control::trajectory::TrajectoryPoint>{});

  m_robot->sendCommand(
      robot::subsystems::ESubsystem::HOLONOMIC_DRIVE_TRAIN,
      robot::subsystems::ESubsystemCommand::
          HOLONOMIC_DRIVE_TRAIN_SET_MOTION_VECTOR,
      robot::subsystems::holonomic_drive_train::HolonomicMotionVector{0.0, 0.0,
                                                                      0.0});
}

std::string TrajectoryTestAuton::getName() { return AUTON_NAME; }

void TrajectoryTestAuton::init(
    std::shared_ptr<robot::Robot>& robot,
    std::shared_ptr<control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;

  // Set up trajectory generator and processor
  control::trajectory::trajectory_generator::TrajectoryGenerator
      trajectory_generator{0.1};
  control::trajectory::trajectory_generator::TrajectoryProcessor
      trajectory_processor{};

  control::trajectory::trajectory_generator::kinematics::
      LinearAccelerationConstraintBuilder
          linear_acceleration_constraint_builder{};
  std::unique_ptr<control::trajectory::trajectory_generator::kinematics::
                      IKinematicConstraint>
      linear_acceleration_constraint{
          linear_acceleration_constraint_builder.withMaxAcceleration(120.0)
              ->build()};
  trajectory_generator.addConstraint(linear_acceleration_constraint);

  control::trajectory::trajectory_generator::kinematics::
      MaxVelocityConstraintBuilder max_velocity_constraint_builder{};
  std::unique_ptr<control::trajectory::trajectory_generator::kinematics::
                      IKinematicConstraint>
      max_velocity_constraint{
          max_velocity_constraint_builder.withMaxVelocity(30.0)->build()};
  trajectory_generator.addConstraint(max_velocity_constraint);

  control::trajectory::trajectory_generator::kinematics::
      CentripetalAccelerationConstraintBuilder
          centripetal_acceleration_constraint_builder{};
  std::unique_ptr<control::trajectory::trajectory_generator::kinematics::
                      IKinematicConstraint>
      centripetal_acceleration_constraint{
          centripetal_acceleration_constraint_builder
              .withMaxCentripetalAcceleration(100)
              ->build()};
  trajectory_generator.addConstraint(centripetal_acceleration_constraint);

  // create test path
  std::unique_ptr<control::trajectory::IPath> path{
      std::make_unique<control::trajectory::QuinticBezierSplinePath>(
          std::vector<control::Point>{
              control::Point{0.0, 0.0}, control::Point{0.0, 20.0},
              control::Point{0.0, 36.0}, control::Point{0.0, 52.0}})};

  trajectory_generator.generateTrajectory(path);

  m_trajectory = trajectory_generator.getTrajectory();
}

void TrajectoryTestAuton::run(
    std::shared_ptr<robot::Robot>& robot,
    std::shared_ptr<control::ControlSystem>& control_system,
    std::shared_ptr<alliance::IAlliance>& alliance,
    std::shared_ptr<rtos::IClock>& clock,
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_alliance = alliance;
  m_clock = clock;
  m_delayer = std::move(delayer);

  setOdomPosition(0, 0, M_PI / 2.0);

  followTrajectory(m_trajectory);
  waitForTrajectory(m_trajectory.back(), 2.0, 3000);
  stopMotion();
}
}  // namespace driftless::auton