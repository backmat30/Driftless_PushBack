#ifndef __RAKE_SUBSYSTEM_HPP__
#define __RAKE_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
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

  /// @brief Subsystem for the rakes
  /// @author Matthew Backman
  class RakeSubsystem : public ASubsystem {
    private:
    std::unique_ptr<IRake> m_rake{};

    public:
    /// @brief Constructs a new rake subsystem
    /// @param rake __std::unique_ptr<IRake>&__ The rake controller used
    RakeSubsystem(std::unique_ptr<IRake>& rake);

    /// @brief Initializes the rake subsystem
    void init() override;

    /// @brief Runs the rake subsystem
    void run() override;

    /// @brief Sends a command to the rake controller
    /// @param command_name __ESubsystemCommand__ The name of the command
    /// @param args __ve_list&__ Any potential arguments for the command
    void command(ESubsystemCommand command_name, va_list& args);

    /// @brief Gets a state of the rake
    /// @param state_name __ESubsystemState__ The state to request
    /// @return __void*__ Pointer to the desired state
    void* state(ESubsystemState state_name);
  };
}}}}
#endif