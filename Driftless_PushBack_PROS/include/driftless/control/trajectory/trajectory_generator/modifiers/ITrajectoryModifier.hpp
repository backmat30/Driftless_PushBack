#ifndef __I_TRAJECTORY_MODIFIER_HPP__
#define __I_TRAJECTORY_MODIFIER_HPP__

#include <vector>

#include "driftless/control/trajectory/TrajectoryPoint.hpp"

namespace driftless {
namespace control {
namespace trajectory {
namespace trajectory_generator {
namespace modifiers {
class ITrajectoryModifier {
 public:
  virtual ~ITrajectoryModifier() = default;

  virtual void applyModifier(std::vector<TrajectoryPoint>& trajectory) = 0;
};
}  // namespace modifiers
}  // namespace trajectory_generator
}  // namespace trajectory
}  // namespace control
}  // namespace driftless
#endif