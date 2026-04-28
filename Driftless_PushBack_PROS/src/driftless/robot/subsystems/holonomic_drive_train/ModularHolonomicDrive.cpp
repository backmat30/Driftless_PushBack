#include "driftless/robot/subsystems/holonomic_drive_train/ModularHolonomicDrive.hpp"

namespace driftless::robot::subsystems::holonomic_drive_train {
void ModularHolonomicDrive::taskLoop(void* params) {
  ModularHolonomicDrive* drive{static_cast<ModularHolonomicDrive*>(params)};

  while (true) {
    drive->taskUpdate();
  }
}

void ModularHolonomicDrive::taskUpdate() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (!m_paused) {
    if (m_control_type == ControlType::VELOCITY) {
      for (auto& module : m_modules) {
        if (module) {
          module->setNormalizedMotionVector(m_current_velocity);
        }
      }
    } else if (m_control_type == ControlType::VOLTAGE) {
      for (auto& module : m_modules) {
        if (module) {
          module->setNormalizedMotionVectorVoltage(m_current_velocity);
        }
      }
    }
  }

  if (m_mutex) {
    m_mutex->give();
  }

  m_delayer->delay(TASK_DELAY);
}

void ModularHolonomicDrive::init() {
  for (auto& module : m_modules) {
    if (module) {
      module->init();
    }
  }
}

void ModularHolonomicDrive::run() {
  for (auto& module : m_modules) {
    if (module) {
      module->run();
    }
  }

  m_task->start(&ModularHolonomicDrive::taskLoop, this);
}

void ModularHolonomicDrive::setMotionVector(
    HolonomicMotionVector motion_vector) {
  setLinearVelocity(motion_vector.x, motion_vector.y);
  setAngularVelocity(motion_vector.angular_velocity);
}

void ModularHolonomicDrive::setLinearVelocity(double x_velocity,
                                              double y_velocity) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = false;

  double velocity{std::sqrt(x_velocity * x_velocity + y_velocity * y_velocity)};
  if (velocity > m_max_linear_velocity) {
    double scale = m_max_linear_velocity / velocity;
    x_velocity *= scale;
    y_velocity *= scale;
  }

  m_current_velocity.x = x_velocity / m_max_linear_velocity;
  m_current_velocity.y = y_velocity / m_max_linear_velocity;

  if (m_mutex) {
    m_mutex->give();
  }
}

void ModularHolonomicDrive::setAngularVelocity(double angular_velocity) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = false;

  if (std::abs(angular_velocity) > m_max_angular_velocity) {
    angular_velocity = (angular_velocity / std::abs(angular_velocity)) *
                       m_max_angular_velocity;
  }

  m_current_velocity.angular_velocity =
      angular_velocity / m_max_angular_velocity;

  if (m_mutex) {
    m_mutex->give();
  }
}

void ModularHolonomicDrive::setNormalizedMotionVector(
    HolonomicMotionVector motion_vector) {
  setNormalizedLinearVelocity(motion_vector.x, motion_vector.y);
  setNormalizedAngularVelocity(motion_vector.angular_velocity);
}

void ModularHolonomicDrive::setNormalizedLinearVelocity(double x_velocity,
                                                        double y_velocity) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = false;

  // Clamp the x and y to [-1, 1]
  if (x_velocity > 1.0) {
    x_velocity = 1.0;
  } else if (x_velocity < -1.0) {
    x_velocity = -1.0;
  }

  if (y_velocity > 1.0) {
    y_velocity = 1.0;
  } else if (y_velocity < -1.0) {
    y_velocity = -1.0;
  }

  m_current_velocity.x = x_velocity;
  m_current_velocity.y = y_velocity;

  if (m_mutex) {
    m_mutex->give();
  }
}

void ModularHolonomicDrive::setNormalizedAngularVelocity(
    double angular_velocity) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = false;

  // Clamp the angular velocity to [-1, 1]
  if (angular_velocity > 1.0) {
    angular_velocity = 1.0;
  } else if (angular_velocity < -1.0) {
    angular_velocity = -1.0;
  }

  m_current_velocity.angular_velocity = angular_velocity;

  if (m_mutex) {
    m_mutex->give();
  }
}

void ModularHolonomicDrive::setWheelVoltage(int wheel, double voltage) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_paused = true;

  if (wheel < 0 || wheel >= m_modules.size()) {
    return;
  }

  m_current_velocity = {0, 0, 0};

  if (m_modules[wheel]) {
    m_modules[wheel]->setRawVoltage(voltage);
  }
  if (m_mutex) {
    m_mutex->give();
  }
}

void ModularHolonomicDrive::setControlType(ControlType control_type) {
  if (m_mutex) {
    m_mutex->take();
  }

  m_control_type = control_type;

  if (m_mutex) {
    m_mutex->give();
  }
}

double ModularHolonomicDrive::getWheelSpeed(int wheel) {
  if (wheel < 0 || wheel >= m_modules.size()) {
    return 0.0;
  }

  if (m_modules[wheel]) {
    return m_modules[wheel]->getSpeed();
  }

  return 0.0;
}

void ModularHolonomicDrive::setModules(
    std::vector<std::unique_ptr<holonomic_drive_module::IHolonomicDriveModule>>&
        modules) {
  m_modules = std::move(modules);
}

void ModularHolonomicDrive::setTask(std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
}

void ModularHolonomicDrive::setDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
}

void ModularHolonomicDrive::setMutex(std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void ModularHolonomicDrive::setMaxLinearVelocity(double max_linear_velocity) {
  m_max_linear_velocity = max_linear_velocity;
}

void ModularHolonomicDrive::setMaxAngularVelocity(double max_angular_velocity) {
  m_max_angular_velocity = max_angular_velocity;
}
}  // namespace driftless::robot::subsystems::holonomic_drive_train