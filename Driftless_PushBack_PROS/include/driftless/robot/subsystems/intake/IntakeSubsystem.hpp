#ifndef __INTAKE_SUBSYSTEM_HPP__
#define __INTAKE_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/intake/IIntake.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for the intake subsystem
/// @author Matthew Backman
namespace intake {
/// @brief Subsystem for an intake
/// @author Matthew Backman
class IntakeSubsystem : public ASubsystem {
  private:
    /// @brief The intake being wrapped
    std::unique_ptr<IIntake> m_intake{};

  public:
    /// @brief Constructs a new IntakeSubsystem
    /// @param intake __std::unique_ptr<IIntake>&__ The intake to be wrapped
    IntakeSubsystem(std::unique_ptr<IIntake>& intake);

    /// @brief Initializes the subsystem
    void init() override;

    /// @brief Runs the subsystem
    void run() override;

    /// @brief Sends a command to the subsystem
    /// @param command_name __ESubsystemCommand__ The command to send
    /// @param args __va_list&__ The arguments for the command
    void command(ESubsystemCommand command_name, va_list& args) override;

    /// @brief Gets a state of the subsystem
    /// @param state_name __ESubsystemState__ The state to get
    /// @return __void*__ The value of the desired state
    void* state(ESubsystemState state_name) override;
};
}  // namespace intake
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif