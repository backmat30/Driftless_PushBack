#include "driftless/robot/subsystems/intake/DirectIntake.hpp"

#include "pros/screen.hpp"
namespace driftless::robot::subsystems::intake {
void DirectIntake::taskLoop(void* params) {
  DirectIntake* intake = static_cast<DirectIntake*>(params);

  while (true) {
    intake->taskUpdate();
  }
}

bool DirectIntake::hasOpposingBlock() {
  bool result{};

  if (m_color_sensor && m_alliance != alliance::EAlliance::NONE) {
    double red{m_color_sensor->getRGB().red};
    double blue{m_color_sensor->getRGB().blue};

    if (m_color_sensor->getProximity() >= 200) {
      switch (m_alliance) {
        case alliance::EAlliance::RED: {
          if (blue > red * 1.25) {
            result = true;
          }
          break;
        }
        case alliance::EAlliance::BLUE: {
          if (red > blue * 1.25) {
            result = true;
          }
          break;
        }
      }
    }
  }

  return result;
}

void DirectIntake::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  // pros::screen::print(
  //     pros::E_TEXT_MEDIUM_CENTER, 5, "dist: %d R: %7.2f B: %7.2f",
  //     m_color_sensor->getProximity(), m_color_sensor->getRGB().red,
  //     m_color_sensor->getRGB().blue);

  // if (m_latest_opposing_block_pos >= -10000) {
  //   pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 6, "latest pos: %7.2f",
  //                       m_latest_opposing_block_pos);
  // }
  // pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 6, "current pos: %7.2f",
  //                     m_front_motors.getPosition());
  // pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 7, "first pos: %7.2f",
  //                     m_first_matchloader_block_pos);

  if (m_running_back_intake) {
    // detect first block and record position
    if (m_color_sensor->getProximity() > 250 &&
        !m_has_first_matchloader_block) {
      m_first_matchloader_block_pos = m_front_motors.getPosition();
      m_has_first_matchloader_block = true;
      // if we have the first block, are not yet ready for the second block, and
      // there is either no block visible or we have moved the front intake far
      // enough, declare that we are ready for block 2
    } else if (m_has_first_matchloader_block &&
               !m_ready_for_second_matchloader_block &&
               (m_front_motors.getPosition() <
                    m_first_matchloader_block_pos - 0.5 ||
                m_color_sensor->getProximity() < 100)) {
      m_ready_for_second_matchloader_block = true;
      // if we are ready for a second block but do not have a second block,
      // check if we can see a second block and update flag
    } else if (m_ready_for_second_matchloader_block &&
               !m_has_second_matchloader_block &&
               m_color_sensor->getProximity() > 200) {
      m_has_second_matchloader_block = true;
      // wait for the front intake to go far enough, then update the flag to
      // direct blocks to the hood
    } else if (m_has_second_matchloader_block &&
               m_front_motors.getPosition() <
                   m_first_matchloader_block_pos - 2.45) {
      m_back_intake_to_hood = true;
    }

    if (m_back_intake_to_hood) {
      m_front_motors.setVoltage(0.0);
      m_front_motors.setCurrentLimit(2.5);

      m_vertical_motors.setVoltage(12.0);
      m_vertical_motors.setCurrentLimit(2.5);
    } else if (m_has_first_matchloader_block &&
               !m_ready_for_second_matchloader_block) {
      m_front_motors.setVoltage(-8.0);
      m_front_motors.setCurrentLimit(2.5);

      m_vertical_motors.setVoltage(-12.0);
      m_vertical_motors.setCurrentLimit(1.0);
    } else if (m_has_second_matchloader_block &&
               m_front_motors.getPosition() >
                   m_first_matchloader_block_pos - 2.45) {
      m_front_motors.setVoltage(-4.0);
      m_front_motors.setCurrentLimit(2.5);

      m_vertical_motors.setVoltage(-10.0);
      m_vertical_motors.setCurrentLimit(1.0);
    } else {
      m_front_motors.setVoltage(0.0);

      m_vertical_motors.setVoltage(-6.0);
      m_vertical_motors.setCurrentLimit(1.0);
    }

    m_intermediary_motors.setVoltage(12.0);
    m_intermediary_motors.setCurrentLimit(2.5);

    m_back_motors.setVoltage(12.0);
    m_back_motors.setCurrentLimit(1.0);
  } else if(!m_color_sort_paused && m_running_forward){
    if (m_running_forward && hasOpposingBlock()) {
      m_latest_opposing_block_pos = m_front_motors.getPosition();
    }
    if (m_front_motors.getPosition() >
            m_latest_opposing_block_pos + COLOR_SORT_DISTANCE_TO_END * 2.0 ||
        m_front_motors.getPosition() <
            m_latest_opposing_block_pos - COLOR_SORT_DISTANCE_TO_END) {
      m_latest_opposing_block_pos = -__DBL_MAX__;
      m_back_motors.setVoltage(-6.0);
      m_back_motors.setCurrentLimit(1.25);
      m_intermediary_motors.setVoltage(12.0);
      m_intermediary_motors.setCurrentLimit(1.5);
    } else if (m_front_motors.getPosition() <
                   m_latest_opposing_block_pos + COLOR_SORT_DISTANCE_TO_END &&
               m_running_forward) {
      m_back_motors.setVoltage(-12.0);
      m_intermediary_motors.setVoltage(-12.0);
      m_intermediary_motors.setCurrentLimit(2.5);
    }
  }

  if (m_mutex) {
    m_mutex->give();
  }

  if (m_delayer) {
    m_delayer->delay(TASK_DELAY);
  }
}

void DirectIntake::init() {
  m_front_motors.init();
  m_intermediary_motors.init();
  m_back_motors.init();
  m_vertical_motors.init();
  m_color_sensor->init();
}

void DirectIntake::run() { m_task->start(&taskLoop, this); }

void DirectIntake::intakeFront(bool reversed) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_running_forward = !reversed;
  m_back_intake_to_hood = false;
  m_has_first_matchloader_block = false;
  m_first_matchloader_block_pos = -__DBL_MAX__;
  m_has_second_matchloader_block = false;
  m_ready_for_second_matchloader_block = false;
  m_running_back_intake = false;

  double voltage = 12.0 * (reversed ? -1.0 : 1.0);
  m_front_motors.setVoltage(voltage);
  m_front_motors.setCurrentLimit(2.5);

  m_vertical_motors.setVoltage(voltage);
  m_vertical_motors.setCurrentLimit(1.5);

  if (m_front_motors.getPosition() >
      m_latest_opposing_block_pos + COLOR_SORT_DISTANCE_TO_END) {
    m_back_motors.setVoltage(reversed ? 12.0 : 6.0);
    m_back_motors.setCurrentLimit(1.25);

    m_intermediary_motors.setVoltage(voltage);
    m_intermediary_motors.setCurrentLimit(1.5);
  }

  if (m_mutex) {
    m_mutex->give();
  }
}

void DirectIntake::intakeBack() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_running_forward = false;
  m_running_back_intake = true;

  if (m_mutex) {
    m_mutex->give();
  }
}

void DirectIntake::intakeBackToHood() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_running_forward = false;
  m_running_back_intake = true;
  m_back_intake_to_hood = true;

  if (m_mutex) {
    m_mutex->give();
  }
}

void DirectIntake::stopIntake() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_running_forward = false;
  m_running_back_intake = false;

  m_front_motors.setVoltage(0.0);
  m_back_motors.setVoltage(0.0);
  m_intermediary_motors.setVoltage(0.0);
  m_vertical_motors.setVoltage(0.0);

  if (m_mutex) {
    m_mutex->give();
  }
}

void DirectIntake::startColorSort(alliance::EAlliance alliance) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_alliance = alliance;
  m_color_sort_paused = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

void DirectIntake::pauseColorSort() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_color_sort_paused = true;

  if (m_mutex) {
    m_mutex->give();
  }
}

void DirectIntake::resumeColorSort() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_color_sort_paused = false;

  if (m_mutex) {
    m_mutex->give();
  }
}

void DirectIntake::deploy() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_back_pistons.extend();

  if (m_mutex) {
    m_mutex->give();
  }
}

void DirectIntake::retract() {
  if (m_mutex) {
    m_mutex->take();
  }

  m_back_pistons.retract();

  if (m_mutex) {
    m_mutex->give();
  }
}

bool DirectIntake::isDeployed() { return m_back_pistons.isExtended(); }

void DirectIntake::setFrontMotors(hal::MotorGroup& motors) {
  m_front_motors = motors;
}

void DirectIntake::setIntermediaryMotors(hal::MotorGroup& motors) {
  m_intermediary_motors = motors;
}

void DirectIntake::setBackMotors(hal::MotorGroup& motors) {
  m_back_motors = motors;
}

void DirectIntake::setBackPistons(hal::PistonGroup& pistons) {
  m_back_pistons = pistons;
}

void DirectIntake::setVerticalMotors(hal::MotorGroup& motors) {
  m_vertical_motors = motors;
}

void DirectIntake::setColorSensor(
    std::unique_ptr<io::IColorSensor>& color_sensor) {
  m_color_sensor = std::move(color_sensor);
}

void DirectIntake::setDelayer(const std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = delayer->clone();
}

void DirectIntake::setMutex(std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void DirectIntake::setTask(std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
}
}  // namespace driftless::robot::subsystems::intake