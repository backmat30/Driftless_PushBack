#ifndef __PNEUMATIC_BRAKE_BUILDER_HPP__
#define __PNEUMATIC_BRAKE_BUILDER_HPP__

#include "driftless/robot/subsystems/brake/PneumaticBrake.hpp"

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
/// @brief Builder class for PneumaticBrake objects
/// @author Matthew Backman
class PneumaticBrakeBuilder {
 private:
  std::unique_ptr<io::IPiston> m_brake_piston{};

 public:
  /// @brief Adds a brake piston to the pneumatic brake
  /// @param brake_piston __std::unique_ptr<io::IPiston>&__ The piston to add
  /// @return __PneumaticBrakeBuilder*__ Pointer to the current builder
  PneumaticBrakeBuilder* withBrakePiston(
      std::unique_ptr<io::IPiston>& brake_piston);

  /// @brief Builds a pneumatic brake
  /// @return __std::unique_ptr<PneumaticBrake>__ The newly build pneumatic brake
  std::unique_ptr<PneumaticBrake> build();
};
}  // namespace brake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif