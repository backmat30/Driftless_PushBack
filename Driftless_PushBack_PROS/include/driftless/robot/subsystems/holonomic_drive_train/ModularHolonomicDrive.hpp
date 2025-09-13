#ifndef __MODULAR_HOLONOMIC_DRIVE_HPP__
#define __MODULAR_HOLONOMIC_DRIVE_HPP__

#include <memory>
#include <vector>

#include "driftless/robot/subsystems/holonomic_drive_train/IHolonomicDrive.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/holonomic_drive_module/IHolonomicDriveModule.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief The namespace for the holonomic drive train subsystem code
/// @author Matthew Backman
namespace holonomic_drive_train {

/// @brief Class representing a modular holonomic drive train
/// @author Matthew Backman
class ModularHolonomicDrive : public IHolonomicDrive {
 private:
  /// @brief The modules controlled by the drive train
  std::vector<std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>>
      m_modules;

 public:
  /// @brief Initializes the modular holonomic drive
  void init() override;

  /// @brief Runs the modular holonomic drive
  void run() override;

  /// @brief Sets the motion vector of the modular holonomic drive
  /// @param motionVector __HolonomicMotionVector__ Vector representing the
  /// desired motion of the robot (robot-centric)
  void setMotionVector(HolonomicMotionVector motionVector) override;

  /// @brief Adds a module to the modular holonomic drive
  /// @param module __unique_ptr<IHolonomicDriveModule>&__ The module to be
  /// added
  void addModule(
      std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>& module);
};
}  // namespace holonomic_drive_train
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif