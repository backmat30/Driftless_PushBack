#include "driftless/control/trajectory/trajectory_follower/TrajectoryFollowerControl.hpp"

namespace driftless::control::trajectory::trajectory_follower {
TrajectoryFollowerControl::TrajectoryFollowerControl(
    std::unique_ptr<ITrajectoryFollower>& trajectory_follower)
    : m_trajectory_follower{std::move(trajectory_follower)}, AControl(EControl::TRAJECTORY_FOLLOWER) {}

void TrajectoryFollowerControl::init() { m_trajectory_follower->init(); }

void TrajectoryFollowerControl::run() { m_trajectory_follower->run(); }

void TrajectoryFollowerControl::pause() { m_trajectory_follower->pause(); }

void TrajectoryFollowerControl::resume() { m_trajectory_follower->resume(); }

void TrajectoryFollowerControl::command(EControlCommand command_name,
                                        va_list& args) {
  switch (command_name) {
    case EControlCommand::FOLLOW_TRAJECTORY: {
      void* temp_robot{va_arg(args, void*)};
      std::shared_ptr<driftless::robot::Robot> robot{
          *static_cast<std::shared_ptr<driftless::robot::Robot>*>(temp_robot)};

      void* temp_trajectory{va_arg(args, void*)};
      std::vector<TrajectoryPoint> trajectory{
          *static_cast<std::vector<TrajectoryPoint>*>(temp_trajectory)};

      m_trajectory_follower->followTrajectory(robot, trajectory);
      break;
    }
  }
}

void* TrajectoryFollowerControl::state(EControlState state_name) {
  void* result{nullptr};

  switch (state_name) {
    case (EControlState::TRAJECTORY_FOLLOWER_TARGET_REACHED): {
      result = new bool(m_trajectory_follower->targetReached());    
    }
  }
  
  return result;
}
}  // namespace driftless::control::trajectory::trajectory_follower