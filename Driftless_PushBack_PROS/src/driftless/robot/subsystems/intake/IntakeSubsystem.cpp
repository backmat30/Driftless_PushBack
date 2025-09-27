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
    case ESubsystemCommand::INTAKE_SET_FRONT_VOLTAGE: {
      double voltage{va_arg(args, double)};
      m_intake->setFrontVoltage(voltage);
      break;
    }
    case ESubsystemCommand::INTAKE_SET_INTERMEDIARY_VOLTAGE: {
      double voltage{va_arg(args, double)};
      m_intake->setIntermediaryVoltage(voltage);
      break;
    }
    case ESubsystemCommand::INTAKE_SET_BACK_VOLTAGE: {
      double voltage{va_arg(args, double)};
      m_intake->setBackVoltage(voltage);
      break;
    }
    case ESubsystemCommand::INTAKE_SET_VERTICAL_VOLTAGE: {
      double voltage{va_arg(args, double)};
      m_intake->setVerticalVoltage(voltage);
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