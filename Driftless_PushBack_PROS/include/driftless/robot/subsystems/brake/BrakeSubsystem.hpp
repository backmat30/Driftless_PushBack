#ifndef __BRAKE_SUBSYSTEM_HPP__
#define __BRAKE_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
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

  /// @brief Subsystem for the brakes
  /// @author Matthew Backman
  class BrakeSubsystem : public ASubsystem {
    private:
    std::unique_ptr<IBrake> m_brake{};

    public:
    /// @brief Constructs a new brake subsystem
    /// @param brake __std::unique_ptr<IBrake>&__ The brake controller used
    BrakeSubsystem(std::unique_ptr<IBrake>& brake);

    /// @brief Initializes the brake subsystem
    void init() override;

    /// @brief Runs the brake subsystem
    void run() override;

    /// @brief Sends a command to the brake controller
    /// @param command_name __ESubsystemCommand__ The name of the command
    /// @param args __ve_list&__ Any potential arguments for the command
    void command(ESubsystemCommand command_name, va_list& args);

    /// @brief Gets a state of the brake
    /// @param state_name __ESubsystemState__ The state to request
    /// @return __void*__ Pointer to the desired state
    void* state(ESubsystemState state_name);
  };
}}}}
#endif