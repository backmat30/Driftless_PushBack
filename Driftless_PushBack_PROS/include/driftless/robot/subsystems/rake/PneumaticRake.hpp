#ifndef __PNEUMATIC_RAKE_HPP__
#define __PNEUMATIC_RAKE_HPP__

#include <memory>

#include "driftless/io/IPiston.hpp"
#include "driftless/robot/subsystems/rake/IRake.hpp"

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

/// @brief Rake object using pneumatics
/// @author Matthew Backman
class PneumaticRake : public IRake {
 private:
  bool m_is_deployed{};

  std::unique_ptr<io::IPiston> m_rake_piston{};

 public:
  /// @brief Initializes the pneumatic rakes
  void init() override;

  /// @brief Runs the pneumatic rakes
  void run() override;

  /// @brief Deploys the pneumatic rakes
  void deploy() override;

  /// @brief Retracts the pneumatic rakes
  void retract() override;

  /// @brief Determines if the rake is deployed
  /// @return __bool__ True of deployed, otherwise false
  bool isDeployed() override;

  /// @brief Sets the piston to use for the rake
  /// @param rake_piston __std::unique_ptr<io::IPiston>&__ The piston to use
  void setRakePiston(std::unique_ptr<io::IPiston>& rake_piston);
};
}  // namespace rake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif