#include "driftless/robot/subsystems/brake/BrakeSubsystem.hpp"

namespace driftless::robot::subsystems::brake {
BrakeSubsystem::BrakeSubsystem(std::unique_ptr<IBrake>& brake)
    : m_brake{std::move(brake)}, ASubsystem{ESubsystem::BRAKE} {}

void BrakeSubsystem::init() { m_brake->init(); }

void BrakeSubsystem::run() { m_brake->run(); }

void BrakeSubsystem::command(ESubsystemCommand command_name, va_list& args) {
  switch (command_name) {
    case ESubsystemCommand::BRAKE_DEPLOY: {
      m_brake->deploy();
      break;
    }
    case ESubsystemCommand::BRAKE_RETRACT: {
      m_brake->retract();
      break;
    }
  }
}

void* BrakeSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};

  switch (state_name) {
    case ESubsystemState::BRAKE_IS_DEPLOYED: {
      result = new bool(m_brake->isDeployed());
      break;
    }
  }

  return result;
}
}  // namespace driftless::robot::subsystems::brake