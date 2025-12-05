#ifndef __HOOD_SUBSYSTEM_HPP__
#define __HOOD_SUBSYSTEM_HPP__

#include <memory>

#include "driftless/robot/subsystems/ASubsystem.hpp"
#include "driftless/robot/subsystems/hood/IHood.hpp"

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for the hood subsystem
/// @author Matthew Backman
namespace hood {
  /// @brief The subsystem for the hood
  class HoodSubsystem : public ASubsystem {
    private:
      std::unique_ptr<IHood> m_hood{};

    public:
      HoodSubsystem(std::unique_ptr<IHood>& hood);

      void init() override;

      void run() override;

      void command(ESubsystemCommand command_name, va_list& args) override;

      void* state(ESubsystemState state_name) override;
  };
}
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif