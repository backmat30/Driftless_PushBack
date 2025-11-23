#ifndef __RAKE_OPERATOR_HPP__
#define __RAKE_OPERATOR_HPP__

#include "driftless/io/IController.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/profiles/IProfile.hpp"
#include "driftless/robot/Robot.hpp"
#include "driftless/robot/subsystems/ESubsystem.hpp"
#include "driftless/robot/subsystems/ESubsystemCommand.hpp"
#include "driftless/robot/subsystems/ESubsystemState.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for operator control management
/// @author Matthew Backman
namespace op_control {
namespace rake {
class RakeOperator {
  private:
    std::shared_ptr<robot::Robot> m_robot{};

    std::shared_ptr<io::IController> m_controller{};

    void updateToggle(EControllerDigital rake_toggle);

  public:
    RakeOperator(std::shared_ptr<io::IController>& controller, std::shared_ptr<robot::Robot>& robot);

    void update(std::unique_ptr<profiles::IProfile>& profile);
};
}  // namespace rake
}  // namespace op_control
}  // namespace driftless

#endif