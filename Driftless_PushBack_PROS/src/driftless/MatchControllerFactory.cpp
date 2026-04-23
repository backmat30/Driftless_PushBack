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
  std::unique_ptr<auton::AAuton> blue_ten_ball_left_auton{
      std::make_unique<auton::BlueTenBallLeftAuton>()};
  lvgl_menu->addAuton(blue_ten_ball_left_auton);
  std::unique_ptr<auton::AAuton> blue_pump_n_dump_auton{
      std::make_unique<auton::BluePumpNDumpAuton>()};
  lvgl_menu->addAuton(blue_pump_n_dump_auton);

  std::unique_ptr<auton::AAuton> blue_control_rush_left_auton{
      std::make_unique<auton::BlueControlRushLeftAuton>()};
  lvgl_menu->addAuton(blue_control_rush_left_auton);
  std::unique_ptr<auton::AAuton> blue_control_rush_auton{
      std::make_unique<auton::BlueControlRushAuton>()};
  lvgl_menu->addAuton(blue_control_rush_auton);

  std::unique_ptr<auton::AAuton> blue_win_point_auton{
      std::make_unique<auton::BlueWPAuton>()};
  lvgl_menu->addAuton(blue_win_point_auton);
  std::unique_ptr<auton::AAuton> blue_win_point_inverse_auton{
      std::make_unique<auton::BlueWPInverseAuton>()};
  lvgl_menu->addAuton(blue_win_point_inverse_auton);

  std::unique_ptr<auton::AAuton> blue_safe_wp{
      std::make_unique<auton::BlueSafeWP>()};
  lvgl_menu->addAuton(blue_safe_wp);
  std::unique_ptr<auton::AAuton> blue_safe_wp_inverse{
      std::make_unique<auton::BlueSafeWPInverseAuton>()};
  lvgl_menu->addAuton(blue_safe_wp_inverse);

  std::unique_ptr<auton::AAuton> orange_pump_n_dump_auton{
      std::make_unique<auton::OrangePumpNDumpAuton>()};
  lvgl_menu->addAuton(orange_pump_n_dump_auton);
  std::unique_ptr<auton::AAuton> orange_ten_ball_right{
      std::make_unique<auton::OrangeTenBallRightAuton>()};
  lvgl_menu->addAuton(orange_ten_ball_right);

  std::unique_ptr<auton::AAuton> orange_control_rush_auton{
      std::make_unique<auton::OrangeControlRushAuton>()};
  lvgl_menu->addAuton(orange_control_rush_auton);
  std::unique_ptr<auton::AAuton> orange_control_rush_right_auton{
      std::make_unique<auton::OrangeControlRushRightAuton>()};
  lvgl_menu->addAuton(orange_control_rush_right_auton);

  std::unique_ptr<auton::AAuton> orange_win_point_auton{
      std::make_unique<auton::OrangeWPAuton>()};
  lvgl_menu->addAuton(orange_win_point_auton);
  std::unique_ptr<auton::AAuton> orange_win_point_inverse{
      std::make_unique<auton::OrangeWPInverseAuton>()};
  lvgl_menu->addAuton(orange_win_point_inverse);

  std::unique_ptr<auton::AAuton> orange_swp_auton{
      std::make_unique<auton::OrangeSWPAuton>()};
  lvgl_menu->addAuton(orange_swp_auton);
  std::unique_ptr<auton::AAuton> orange_swp_inverse{
      std::make_unique<auton::OrangeSWPInverseAuton>()};
  lvgl_menu->addAuton(orange_swp_inverse);

  std::unique_ptr<auton::AAuton> orange_max_point_auton{
      std::make_unique<auton::OrangeMaxPointAuton>()};
  lvgl_menu->addAuton(orange_max_point_auton);

  // skills
  std::unique_ptr<auton::AAuton> blue_skills_auton{
      std::make_unique<auton::BlueSkillsAuton>()};
  lvgl_menu->addAuton(blue_skills_auton);

  std::unique_ptr<auton::AAuton> orange_skills_auton{
      std::make_unique<auton::OrangeSkillsAuton>()};
  lvgl_menu->addAuton(orange_skills_auton);

  // add configs
  std::unique_ptr<config::IConfig> blue_config{
      std::make_unique<config::BlueConfig>()};
  lvgl_menu->addConfig(blue_config);
  std::unique_ptr<config::IConfig> orange_config{
      std::make_unique<config::OrangeConfig>()};
  lvgl_menu->addConfig(orange_config);

  // add profiles
  std::unique_ptr<profiles::IProfile> ethan_profile{
      std::make_unique<driftless::profiles::EthanProfile>()};
  lvgl_menu->addProfile(ethan_profile);
  std::unique_ptr<profiles::IProfile> asher_profile{
      std::make_unique<profiles::AsherProfile>()};
  lvgl_menu->addProfile(asher_profile);

  // create RTOS
  std::shared_ptr<rtos::IClock> clock{
      std::make_unique<pros_adapters::ProsClock>()};
  std::unique_ptr<rtos::IDelayer> delayer{
      std::make_unique<pros_adapters::ProsDelayer>()};

  // create and send out the match controller
  return MatchController{lvgl_menu, clock, delayer};
}
}  // namespace driftless