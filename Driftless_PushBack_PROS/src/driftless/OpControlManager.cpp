#include "driftless/OpControlManager.hpp"

namespace driftless {
// constructor
OpControlManager::OpControlManager(
    const std::shared_ptr<rtos::IClock>& clock,
    const std::unique_ptr<rtos::IDelayer>& delayer)
    : m_clock{clock}, m_delayer{delayer->clone()} {}

// sets the controller's profile
void OpControlManager::setProfile(
    std::unique_ptr<profiles::IProfile>& profile) {
  m_profile = std::move(profile);
}

void OpControlManager::setAlliance(
    std::shared_ptr<alliance::IAlliance>& alliance) {
  m_alliance = alliance;
}

void OpControlManager::init(
    std::shared_ptr<control::ControlSystem> control_system,
    std::shared_ptr<driftless::processes::ProcessSystem> process_system,
    std::shared_ptr<io::IController> controller,
    std::shared_ptr<robot::Robot> robot) {}
void OpControlManager::run(
    std::shared_ptr<control::ControlSystem> control_system,
    std::shared_ptr<driftless::processes::ProcessSystem> process_system,
    std::shared_ptr<io::IController> controller,
    std::shared_ptr<robot::Robot> robot) {
  // pause control system to allow operator takeover
  control_system->pause();
  process_system->resumeAll();

  // set subsystems to driver control


  // variable to hold time for delayer
  uint32_t current_time{};

  // task loop
  while (true) {
    // grabs time for delayer
    current_time = m_clock->getTime();

    // updates all subsystems
       

    // delay until 10 seconds after loop start
    // keeps time per loop consistent rather than delaying 10 seconds AFTER
    // commands
    m_delayer->delayUntil(current_time + CONTROL_DELAY);
  }
}
}  // namespace driftless