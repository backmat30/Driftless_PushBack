#include "driftless/control/motion/MotionControl.hpp"

#include "pros/screen.hpp"
namespace driftless {
namespace control {
namespace motion {
MotionControl::MotionControl(
    std::unique_ptr<driftless::control::motion::IDriveStraight>& drive_straight,
    std::unique_ptr<driftless::control::motion::IGoToPoint>& go_to_point,
    std::unique_ptr<driftless::control::motion::IGoToPose>& go_to_pose,
    std::unique_ptr<driftless::control::motion::ITurn>& turn)
    : AControl{EControl::MOTION},
      m_drive_straight{std::move(drive_straight)},
      m_go_to_point{std::move(go_to_point)},
      m_go_to_pose{std::move(go_to_pose)},
      m_turn{std::move(turn)} {}

void MotionControl::init() {
  m_drive_straight->init();
  m_go_to_point->init();
  m_go_to_pose->init();
  m_turn->init();
}

void MotionControl::run() {
  m_drive_straight->run();
  m_go_to_point->run();
  m_go_to_pose->run();
  m_turn->run();
}

void MotionControl::pause() {
  switch (m_motion_type) {
    case EMotionType::DRIVE_STRAIGHT:
      m_drive_straight->pause();
      break;
    case EMotionType::GO_TO_POINT:
      m_go_to_point->pause();
      break;
    case EMotionType::GO_TO_POSE:
      m_go_to_pose->pause();
      break;
    case EMotionType::TURN:
      m_turn->pause();
      break;
  }
}

void MotionControl::resume() {
  switch (m_motion_type) {
    case EMotionType::DRIVE_STRAIGHT:
      m_drive_straight->resume();
      break;
    case EMotionType::GO_TO_POINT:
      m_go_to_point->resume();
      break;
    case EMotionType::GO_TO_POSE:
      m_go_to_pose->resume();
    case EMotionType::TURN:
      m_turn->resume();
      break;
  }
}

void MotionControl::command(EControlCommand command_name, va_list& args) {
  if (command_name == EControlCommand::DRIVE_STRAIGHT) {
    if (m_motion_type != EMotionType::DRIVE_STRAIGHT) {
      pause();
      m_motion_type = EMotionType::DRIVE_STRAIGHT;
    }

    void* temp_robot{va_arg(args, void*)};
    std::shared_ptr<driftless::robot::Robot> robot{
        *static_cast<std::shared_ptr<driftless::robot::Robot>*>(temp_robot)};
    double velocity{va_arg(args, double)};
    double distance{va_arg(args, double)};
    double theta{va_arg(args, double)};

    m_drive_straight->driveStraight(robot, velocity, distance, theta);

  } else if (command_name == EControlCommand::GO_TO_POINT) {
    if (m_motion_type != EMotionType::GO_TO_POINT) {
      pause();
      m_motion_type = EMotionType::GO_TO_POINT;
    }

    void* temp_robot{va_arg(args, void*)};
    std::shared_ptr<driftless::robot::Robot> robot{
        *static_cast<std::shared_ptr<driftless::robot::Robot>*>(temp_robot)};
    double velocity{va_arg(args, double)};
    double x{va_arg(args, double)};
    double y{va_arg(args, double)};
    double theta{va_arg(args, double)};
    Point point{x, y, theta};

    m_go_to_point->goToPoint(robot, velocity, point);
  } else if (command_name == EControlCommand::GO_TO_POSE) {
    if (m_motion_type != EMotionType::GO_TO_POSE) {
      pause();
      m_motion_type = EMotionType::GO_TO_POSE;
    }

    void* temp_robot{va_arg(args, void*)};
    std::shared_ptr<driftless::robot::Robot> robot{
        *static_cast<std::shared_ptr<driftless::robot::Robot>*>(temp_robot)};
    double velocity{va_arg(args, double)};
    double angular_velocity{va_arg(args, double)};
    double linear_acceleration{va_arg(args, double)};
    double x{va_arg(args, double)};
    double y{va_arg(args, double)};
    double theta{va_arg(args, double)};
    Point point{x, y, theta};

    m_go_to_pose->goToPose(robot, velocity, angular_velocity,
                           linear_acceleration, point);

  } else if (command_name == EControlCommand::TURN_TO_ANGLE) {
    if (m_motion_type != EMotionType::TURN) {
      pause();
      m_motion_type = EMotionType::TURN;
    }

    void* temp_robot{va_arg(args, void*)};
    std::shared_ptr<driftless::robot::Robot> robot{
        *static_cast<std::shared_ptr<driftless::robot::Robot>*>(temp_robot)};
    double velocity{va_arg(args, double)};
    double theta{va_arg(args, double)};
    ETurnDirection direction{va_arg(args, ETurnDirection)};

    m_turn->turnToAngle(robot, velocity, theta, direction);

  } else if (command_name == EControlCommand::TURN_TO_POINT) {
    if (m_motion_type != EMotionType::TURN) {
      pause();
      m_motion_type = EMotionType::TURN;
    }

    void* temp_robot{va_arg(args, void*)};
    std::shared_ptr<driftless::robot::Robot> robot{
        *static_cast<std::shared_ptr<driftless::robot::Robot>*>(temp_robot)};
    double velocity{va_arg(args, double)};
    double x{va_arg(args, double)};
    double y{va_arg(args, double)};
    ETurnDirection direction{va_arg(args, ETurnDirection)};

    Point point{x, y};
    m_turn->turnToPoint(robot, velocity, point, direction);

  } else if (command_name == EControlCommand::DRIVE_STRAIGHT_SET_VELOCITY) {
    double velocity{va_arg(args, double)};
    m_drive_straight->setVelocity(velocity);

  } else if (command_name == EControlCommand::GO_TO_POINT_SET_VELOCITY) {
    double velocity{va_arg(args, double)};
    m_go_to_point->setVelocity(velocity);
  } else if (command_name == EControlCommand::GO_TO_POSE_SET_VELOCITY) {
    double velocity{va_arg(args, double)};
    m_go_to_pose->setVelocity(velocity);
  } else if (command_name == EControlCommand::GO_TO_POSE_SET_ANGULAR_VELOCITY) {
    double angular_velocity{va_arg(args, double)};
    m_go_to_pose->setAngularVelocity(angular_velocity);
  }
}

void* MotionControl::state(EControlState state_name) {
  void* result{};
  if (state_name == EControlState::DRIVE_STRAIGHT_TARGET_REACHED) {
    result = new bool{m_drive_straight->targetReached()};
  } else if (state_name == EControlState::GO_TO_POINT_TARGET_REACHED) {
    result = new bool{m_go_to_point->targetReached()};
  } else if (state_name == EControlState::GO_TO_POSE_TARGET_REACHED) {
    result = new bool{m_go_to_pose->targetReached()};
  } else if (state_name == EControlState::TURN_TARGET_REACHED) {
    result = new bool{m_turn->targetReached()};
  }

  return result;
}
}  // namespace motion
}  // namespace control
}  // namespace driftless