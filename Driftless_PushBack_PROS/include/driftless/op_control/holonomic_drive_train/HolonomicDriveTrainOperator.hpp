#ifndef __HOLONOMIC_DRIVE_TRAIN_OPERATOR_HPP__
#define __HOLONOMIC_DRIVE_TRAIN_OPERATOR_HPP__

#include <memory>
#include <cmath>

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerAnalog.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"
#include "driftless/robot/subsystems/holonomic_drive_train/HolonomicMotionVector.hpp"

namespace driftless {
namespace op_control {
namespace holonomic_drive_train {

class HolonomicDriveTrainOperator {
 private:
  std::shared_ptr<io::IController> m_controller{};

  std::shared_ptr<robot::Robot> m_robot{};

  void updateDriveMotionVector(EControllerAnalog forward,
                               EControllerAnalog strafe,
                               EControllerAnalog turn);

 public:
  HolonomicDriveTrainOperator(
      const std::shared_ptr<io::IController>& controller,
      const std::shared_ptr<robot::Robot>& robot);

  void setDriveMotionVector(const std::unique_ptr<profiles::IProfile>& profile);
};
}  // namespace holonomic_drive_train
}  // namespace op_control
}  // namespace driftless

#endif