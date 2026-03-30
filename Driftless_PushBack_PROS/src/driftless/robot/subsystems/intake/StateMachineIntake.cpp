#include "driftless/robot/subsystems/intake/StateMachineIntake.hpp"

#include "pros/screen.hpp"
namespace driftless::robot::subsystems::intake {
void StateMachineIntake::taskLoop(void* params) {
  StateMachineIntake* intake = static_cast<StateMachineIntake*>(params);

  while (true) {
    intake->taskUpdate();
  }
}

bool StateMachineIntake::hasBlock() {
  bool result{};

  if (m_color_sensor) {
    if (m_color_sensor->getProximity() >= 200) {
      result = true;
    }
  }
  return result;
}

bool StateMachineIntake::hasOpposingBlock() {
  bool result{};

  if (m_color_sensor && m_alliance != alliance::EAlliance::NONE) {
    double red{m_color_sensor->getRGB().red};
    double blue{m_color_sensor->getRGB().blue};

    if (hasBlock()) {
      switch (m_alliance) {
        case alliance::EAlliance::RED: {
          if (blue > red * 1.2) {
            result = true;
          }
          break;
        }
        case alliance::EAlliance::BLUE: {
          if (red > blue * 1.2) {
            result = true;
          }
          break;
        }
      }
    }
  }

  return result;
}

bool StateMachineIntake::hasAllianceBlock() {
  bool result{};

  if (m_color_sort_paused) {
    if (hasBlock()) {
      result = true;
    }
  } else if (m_color_sensor) {
    double red{m_color_sensor->getRGB().red};
    double blue{m_color_sensor->getRGB().blue};

    if (hasBlock()) {
      switch (m_alliance) {
        case alliance::EAlliance::BLUE: {
          if (blue > red * 1.2) {
            result = true;
          }
          break;
        }
        case alliance::EAlliance::RED: {
          if (red > blue * 1.2) {
            result = true;
          }
          break;
        }
      }
    }
  }

  return result;
}

void StateMachineIntake::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (m_states[static_cast<int>(current_state)]) {
    m_states[static_cast<int>(current_state)]->update(*this);
  }

  if (m_mutex) {
    m_mutex->give();
  }

  if (m_delayer) {
    m_delayer->delay(TASK_DELAY);
  }
}

void StateMachineIntake::init() {
  m_front_motors.init();
  m_intermediary_motors.init();
  m_back_motors.init();
  m_vertical_motors.init();
  m_color_sensor->init();
}

void StateMachineIntake::run() { m_task->start(&taskLoop, this); }

void StateMachineIntake::intakeFront(double voltage) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_desired_voltage = std::abs(voltage);
  if (voltage > 0.0) {
    current_state = intake_states::EIntakeStates::FRONT_IN;
  } else {
    current_state = intake_states::EIntakeStates::FRONT_OUT;
  }

  if (m_mutex) {
    m_mutex->give();
  }
}

void StateMachineIntake::intakeBack() {
  if (m_mutex) {
    m_mutex->take();
  }

  current_state = intake_states::EIntakeStates::BACK_IN_BOTTOM;

  if (m_mutex) {
    m_mutex->give();
  }
}

void StateMachineIntake::intakeBackToHood() {
  if (m_mutex) {
    m_mutex->take();
  }

  current_state = intake_states::EIntakeStates::BACK_IN_TOP;

  if (m_mutex) {
    m_mutex->give();
  }
}

void StateMachineIntake::stopIntake() {
  if (m_mutex) {
    m_mutex->take();
  }

  current_state = intake_states::EIntakeStates::IDLE;

  if (m_mutex) {
    m_mutex->give();
  }
}

void StateMachineIntake::startColorSort(alliance::EAlliance alliance) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_alliance = alliance;
  m_color_sort_paused = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

void StateMachineIntake::pauseColorSort() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_color_sort_paused = true;

  if (m_mutex) {
    m_mutex->give();
  }
}

void StateMachineIntake::resumeColorSort() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_color_sort_paused = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

void StateMachineIntake::deploy() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_back_pistons.extend();

  if (m_mutex) {
    m_mutex->give();
  }
}

void StateMachineIntake::retract() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_back_pistons.retract();

  if (m_mutex) {
    m_mutex->give();
  }
}

bool StateMachineIntake::isDeployed() { return m_back_pistons.isExtended(); }

void StateMachineIntake::setFrontMotorVoltage(double voltage) {
  m_front_motors.setVoltage(voltage);
}

void StateMachineIntake::setFrontMotorCurrentLimit(double amps) {
  m_front_motors.setCurrentLimit(amps);
}

void StateMachineIntake::setIntermediaryMotorVoltage(double voltage) {
  m_intermediary_motors.setVoltage(voltage);
}

void StateMachineIntake::setIntermediaryMotorCurrentLimit(double amps) {
  m_intermediary_motors.setCurrentLimit(amps);
}

void StateMachineIntake::setBackMotorVoltage(double voltage) {
  m_back_motors.setVoltage(voltage);
}

void StateMachineIntake::setBackMotorCurrentLimit(double amps) {
  m_back_motors.setCurrentLimit(amps);
}

void StateMachineIntake::setVerticalMotorVoltage(double voltage) {
  m_vertical_motors.setVoltage(voltage);
}

void StateMachineIntake::setVerticalMotorCurrentLimit(double amps) {
  m_vertical_motors.setCurrentLimit(amps);
}

double StateMachineIntake::getFrontMotorPosition() {
  return m_front_motors.getPosition();
}

double StateMachineIntake::getColorSensorPosition() {
  return COLOR_SORT_DISTANCE_TO_END;
}

bool StateMachineIntake::isColorSortPaused() { return m_color_sort_paused; }

double StateMachineIntake::getDesiredVoltage() { return m_desired_voltage; }

void StateMachineIntake::setFrontMotors(hal::MotorGroup& motors) {
  m_front_motors = motors;
}

void StateMachineIntake::setIntermediaryMotors(hal::MotorGroup& motors) {
  m_intermediary_motors = motors;
}

void StateMachineIntake::setBackMotors(hal::MotorGroup& motors) {
  m_back_motors = motors;
}

void StateMachineIntake::setBackPistons(hal::PistonGroup& pistons) {
  m_back_pistons = pistons;
}

void StateMachineIntake::setVerticalMotors(hal::MotorGroup& motors) {
  m_vertical_motors = motors;
}

void StateMachineIntake::setColorSensor(
    std::unique_ptr<io::IColorSensor>& color_sensor) {
  m_color_sensor = std::move(color_sensor);
}

void StateMachineIntake::setDelayer(
    const std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
}

void StateMachineIntake::setMutex(std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void StateMachineIntake::setTask(std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
}
}  // namespace driftless::robot::subsystems::intake