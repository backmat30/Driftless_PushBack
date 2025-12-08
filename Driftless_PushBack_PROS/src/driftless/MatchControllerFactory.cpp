#include "driftless/MatchControllerFactory.hpp"

namespace driftless {
driftless::MatchController MatchControllerFactory::createMatchController() {
  // the display menu
  std::unique_ptr<menu::IMenu> lvgl_menu{std::make_unique<menu::MenuAdapter>()};
  // add alliances
  std::shared_ptr<alliance::IAlliance> blue_alliance{
      std::make_shared<alliance::BlueAlliance>()};
  lvgl_menu->addAlliance(blue_alliance);

  std::shared_ptr<alliance::IAlliance> red_alliance{
      std::make_shared<alliance::RedAlliance>()};
  lvgl_menu->addAlliance(red_alliance);

  std::shared_ptr<alliance::IAlliance> skills_alliance{
      std::make_shared<alliance::SkillsAlliance>()};
  lvgl_menu->addAlliance(skills_alliance);

  // add auton routes
  std::unique_ptr<auton::AAuton> blue_skills_auton{
      std::make_unique<auton::BlueSkillsAuton>()};
  lvgl_menu->addAuton(blue_skills_auton);

  // add configs
  std::unique_ptr<config::IConfig> blue_config{
      std::make_unique<config::BlueConfig>()};
  lvgl_menu->addConfig(blue_config);

  // add profiles
  std::unique_ptr<profiles::IProfile> ethan_profile{
      std::make_unique<driftless::profiles::EthanProfile>()};
  lvgl_menu->addProfile(ethan_profile);

  // create RTOS
  std::shared_ptr<rtos::IClock> clock{
      std::make_unique<pros_adapters::ProsClock>()};
  std::unique_ptr<rtos::IDelayer> delayer{
      std::make_unique<pros_adapters::ProsDelayer>()};

  // create and send out the match controller
  return MatchController{lvgl_menu, clock, delayer};
}
}  // namespace driftless