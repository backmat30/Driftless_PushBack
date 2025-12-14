#ifndef __PNEUMATIC_BRAKE_HPP__
#define __PNEUMATIC_BRAKE_HPP__

#include <memory>

#include "driftless/io/IPiston.hpp"
#include "driftless/robot/subsystems/brake/IBrake.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief Namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for the brake subsystem
/// @author Matthew Backman
namespace brake {

/// @brief Brake object using pneumatics
/// @author Matthew Backman
class PneumaticBrake : public IBrake {
 private:
  bool m_is_deployed{};

  std::unique_ptr<io::IPiston> m_brake_piston{};

 public:
  /// @brief Initializes the pneumatic brakes
  void init() override;

  /// @brief Runs the pneumatic brakes
  void run() override;

  /// @brief Deploys the pneumatic brakes
  void deploy() override;

  /// @brief Retracts the pneumatic brakes
  void retract() override;

  /// @brief Determines if the brake is deployed
  /// @return __bool__ True of deployed, otherwise false
  bool isDeployed() override;

  /// @brief Sets the piston to use for the brake
  /// @param brake_piston __std::unique_ptr<io::IPiston>&__ The piston to use
  void setBrakePiston(std::unique_ptr<io::IPiston>& brake_piston);
};
}  // namespace brake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif