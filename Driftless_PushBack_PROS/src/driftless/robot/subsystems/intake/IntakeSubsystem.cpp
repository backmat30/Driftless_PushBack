#include "driftless/robot/subsystems/intake/IntakeSubsystem.hpp"

namespace driftless::robot::subsystems::intake {
IntakeSubsystem::IntakeSubsystem(std::unique_ptr<IIntake>& intake)
    : ASubsystem{ESubsystem::INTAKE}, m_intake{std::move(intake)} {}

void IntakeSubsystem::init() {
  if (m_intake) {
    m_intake->init();
  }
}

void IntakeSubsystem::run() {
  if (m_intake) {
    m_intake->run();
  }
}

void IntakeSubsystem::command(ESubsystemCommand command_name, va_list& args) {
  switch (command_name) {
    case ESubsystemCommand::INTAKE_FRONT_IN: {
      m_intake->intakeFront(false);
      break;
    }
    case ESubsystemCommand::INTAKE_FRONT_OUT: {
      m_intake->intakeFront(true);
      break;
    }
    case ESubsystemCommand::INTAKE_BACK_IN: {
      m_intake->intakeBack();
      break;
    }
    case ESubsystemCommand::INTAKE_STOP_MOTION: {
      m_intake->stopIntake();
      break;
    }
    case ESubsystemCommand::INTAKE_DEPLOY_ARMS: {
      m_intake->deploy();
      break;
    }
    case ESubsystemCommand::INTAKE_RETRACT_ARMS: {
      m_intake->retract();
      break;
    }
    case ESubsystemCommand::INTAKE_START_COLOR_SORT: {
      alliance::EAlliance alliance{va_arg(args, alliance::EAlliance)};
      m_intake->startColorSort(alliance);
      break;
    }
    case ESubsystemCommand::INTAKE_PAUSE_COLOR_SORT: {
      m_intake->pauseColorSort();
      break;
    }
    case ESubsystemCommand::INTAKE_RESUME_COLOR_SORT: {
      m_intake->resumeColorSort();
      break;
    }
  }
}

void* IntakeSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};
  switch (state_name) {
    case ESubsystemState::INTAKE_ARE_ARMS_DEPLOYED:
      result = new bool(m_intake->isDeployed());
      break;
  }
  return result;
}
}  // namespace driftless::robot::subsystems::intake