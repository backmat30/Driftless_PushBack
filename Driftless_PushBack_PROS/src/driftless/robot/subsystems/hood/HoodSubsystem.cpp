#include "driftless/robot/subsystems/hood/HoodSubsystem.hpp"

namespace driftless::robot::subsystems::hood {
HoodSubsystem::HoodSubsystem(std::unique_ptr<IHood>& hood)
    : ASubsystem{ESubsystem::HOOD}, m_hood{std::move(hood)} {}

void HoodSubsystem::init() { m_hood->init(); }

void HoodSubsystem::run() { m_hood->run(); }

void HoodSubsystem::command(ESubsystemCommand command_name, va_list& args) {
  switch (command_name) {
    case ESubsystemCommand::HOOD_SET_VOLTAGE: {
      double voltage{va_arg(args, double)};
      m_hood->setVoltage(voltage);
      break;
    }
    case ESubsystemCommand::HOOD_OPEN_GATE: {
      m_hood->open();
      break;
    }
    case ESubsystemCommand::HOOD_CLOSE_GATE: {
      m_hood->close();
      break;
    }
    case ESubsystemCommand::HOOD_TOGGLE_GATE: {
      m_hood->toggleOpen();
      break;
    }
    case ESubsystemCommand::HOOD_RAISE: {
      m_hood->raise();
      break;
    }
    case ESubsystemCommand::HOOD_LOWER: {
      m_hood->lower();
      break;
    }
    case ESubsystemCommand::HOOD_TOGGLE_RAISED: {
      m_hood->toggleRaised();
      break;
    }
    case ESubsystemCommand::HOOD_EXTEND_DESCORE: {
      m_hood->extendDescore();
      break;
    }
    case ESubsystemCommand::HOOD_RETRACT_DESCORE: {
      m_hood->retractDescore();
      break;
    }
    case ESubsystemCommand::HOOD_TOGGLE_DESCORE: {
      m_hood->toggleDescore();
      break;
    }
    case ESubsystemCommand::HOOD_BUMP_UP: {
      m_hood->bumpUp();
      break;
    }
    case ESubsystemCommand::HOOD_BUMP_DOWN: {
      m_hood->bumpDown();
      break;
    }
    case ESubsystemCommand::HOOD_TOGGLE_BUMP: {
      m_hood->toggleBump();
      break;
    }
    case ESubsystemCommand::HOOD_SET_CURRENT_LIMIT: {
      double amps{va_arg(args, double)};
      m_hood->setCurrentLimit(amps);
      break;
    }
  }
}

void* HoodSubsystem::state(ESubsystemState state_name) {
  void* state{nullptr};

  switch (state_name) {
    case ESubsystemState::HOOD_IS_RAISED: {
      state = new bool{m_hood->isRaised()};
      break;
    }
    case ESubsystemState::HOOD_IS_BUMPED: {
      state = new bool{m_hood->isBumped()};
      break;
    }
    case ESubsystemState::HOOD_IS_OPEN: {
      state = new bool{m_hood->isOpen()};
      break;
    }
  }

  return state;
}
}  // namespace driftless::robot::subsystems::hood