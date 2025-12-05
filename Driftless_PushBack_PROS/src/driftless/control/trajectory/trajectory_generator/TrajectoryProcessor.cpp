#include "driftless/control/trajectory/trajectory_generator/TrajectoryProcessor.hpp"

namespace driftless::control::trajectory::trajectory_generator {
std::vector<TrajectoryPoint> TrajectoryProcessor::process(
    const std::vector<TrajectoryPoint>& current_trajectory,
    const std::vector<std::unique_ptr<modifiers::ITrajectoryModifier>>&
        modifier_list) {
  std::vector<TrajectoryPoint> output{current_trajectory};

  for (auto& modifier : modifier_list) {
    modifier->applyModifier(output);
  }

  return output;
}
}  // namespace driftless::control::trajectory::trajectory_generator