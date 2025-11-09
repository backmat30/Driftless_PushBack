#include "driftless/robot/subsystems/odometry/OdometrySubsystem.hpp"

namespace driftless {
namespace robot {
namespace subsystems {
namespace odometry {
OdometrySubsystem::OdometrySubsystem(
    std::unique_ptr<IPositionTracker>& position_tracker)
    : ASubsystem{ESubsystem::ODOMETRY},
      m_position_tracker{std::move(position_tracker)} {}

void OdometrySubsystem::init() {
  if (m_position_tracker) {
    m_position_tracker->init();
  }
}

void OdometrySubsystem::run() {
  if (m_position_tracker) {
    m_position_tracker->run();
  }
}

void OdometrySubsystem::command(ESubsystemCommand command_name, va_list& args) {
  if (command_name == ESubsystemCommand::ODOMETRY_SET_POSITION) {
    if (m_position_tracker) {
      Position position{va_arg(args, double), va_arg(args, double),
                        va_arg(args, double)};
      m_position_tracker->setPosition(position);
    }
  } else if (command_name == ESubsystemCommand::ODOMETRY_SET_X) {
    if (m_position_tracker) {
      double x{va_arg(args, double)};
      m_position_tracker->setX(x);
    }
  } else if (command_name == ESubsystemCommand::ODOMETRY_SET_Y) {
    if (m_position_tracker) {
      double y{va_arg(args, double)};
      m_position_tracker->setY(y);
    }
  } else if (command_name == ESubsystemCommand::ODOMETRY_SET_THETA) {
    if (m_position_tracker) {
      double theta{va_arg(args, double)};
      m_position_tracker->setTheta(theta);
    }
  }
}

void* OdometrySubsystem::state(ESubsystemState state_name) {
  void* result{};

  if (state_name == ESubsystemState::ODOMETRY_GET_POSITION) {
    if (m_position_tracker) {
      Position* position{new Position{m_position_tracker->getPosition()}};
      result = position;
    }
  }

  return result;
}
}  // namespace odometry
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless