#include "driftless/robot/subsystems/rake/RakeSubsystem.hpp"

namespace driftless::robot::subsystems::rake {
RakeSubsystem::RakeSubsystem(std::unique_ptr<IRake>& rake)
    : m_rake{std::move(rake)}, ASubsystem{ESubsystem::BRAKE} {}

void RakeSubsystem::init() { m_rake->init(); }

void RakeSubsystem::run() { m_rake->run(); }

void RakeSubsystem::command(ESubsystemCommand command_name, va_list& args) {
  switch (command_name) {
    case ESubsystemCommand::BRAKE_DEPLOY: {
      m_rake->deploy();
      break;
    }
    case ESubsystemCommand::BRAKE_RETRACT: {
      m_rake->retract();
      break;
    }
  }
}

void* RakeSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};

  switch (state_name) {
    case ESubsystemState::BRAKE_IS_DEPLOYED: {
      result = new bool(m_rake->isDeployed());
      break;
    }
  }

  return result;
}
}  // namespace driftless::robot::subsystems::rake