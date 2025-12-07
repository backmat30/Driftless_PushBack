#include "driftless/auton/BlueSkillsAuton.hpp"

namespace driftless::auton {
void BlueSkillsAuton::init(
    std::shared_ptr<robot::Robot>& robot,
    std::shared_ptr<control::ControlSystem>& control_system) {
  m_robot = robot;
  m_control_system = control_system;
}

void BlueSkillsAuton::run(
    std::shared_ptr<driftless::robot::Robot>& robot,
    std::shared_ptr<driftless::control::ControlSystem>& control_system,
    std::shared_ptr<driftless::alliance::IAlliance>& alliance,
    std::shared_ptr<rtos::IClock>& clock,
    std::unique_ptr<rtos::IDelayer>& delayer) {
      m_robot = robot;
      m_control_system = control_system;
      m_alliance = alliance;
      m_clock = clock;
      m_delayer = delayer->clone();

      setOdomPosition(0, 0, M_PI / 2);
    }
}  // namespace driftless::auton