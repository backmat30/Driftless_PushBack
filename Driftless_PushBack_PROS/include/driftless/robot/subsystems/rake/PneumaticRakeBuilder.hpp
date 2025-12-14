#ifndef __PNEUMATIC_RAKE_BUILDER_HPP__
#define __PNEUMATIC_RAKE_BUILDER_HPP__

#include "driftless/robot/subsystems/rake/PneumaticRake.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief Namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for the rake subsystem
/// @author Matthew Backman
namespace rake {
/// @brief Builder class for PneumaticRake objects
/// @author Matthew Backman
class PneumaticRakeBuilder {
 private:
  std::unique_ptr<io::IPiston> m_rake_piston{};

 public:
  /// @brief Adds a rake piston to the pneumatic rake
  /// @param rake_piston __std::unique_ptr<io::IPiston>&__ The piston to add
  /// @return __PneumaticRakeBuilder*__ Pointer to the current builder
  PneumaticRakeBuilder* withRakePiston(
      std::unique_ptr<io::IPiston>& rake_piston);

  /// @brief Builds a pneumatic rake
  /// @return __std::unique_ptr<PneumaticRake>__ The newly build pneumatic rake
  std::unique_ptr<PneumaticRake> build();
};
}  // namespace rake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless

#endif