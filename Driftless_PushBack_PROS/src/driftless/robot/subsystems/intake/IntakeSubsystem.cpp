#include "driftless/robot/subsystems/intake/IntakeSubsystem.hpp"

namespace driftless::robot::subsystems::intake {
IntakeSubsystem::IntakeSubsystem(ESubsystem subsystem_name,
                                 std::unique_ptr<IIntake>& intake)
    : ASubsystem{subsystem_name}, m_intake{std::move(intake)} {}

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
    case ESubsystemCommand::INTAKE_SET_VOLTAGE:
      double voltage{va_arg(args, double)};
      m_intake->setVoltage(voltage);
      break;
    case ESubsystemCommand::INTAKE_DEPLOY:
      m_intake->deploy();
      break;
    case ESubsystemCommand::INTAKE_RETRACT:
      m_intake->retract();
      break;
  }
}

void* IntakeSubsystem::state(ESubsystemState state_name) {
  void* result{nullptr};

  return result;
}
}  // namespace driftless::robot::subsystems::intake