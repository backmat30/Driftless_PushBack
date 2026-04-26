#ifndef __ORANGE_SKILLS_AUTON_HPP__
#define __ORANGE_SKILLS_AUTON_HPP__

#include "driftless/auton/AAuton.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for autonomous routines
/// @author Matthew Backman
namespace auton {

/// @brief Class representing the autonomous routine for the orange robot's
/// skills route
/// @author Matthew Backman
class OrangeSkillsAuton : public AAuton {
 private:
  static constexpr double MAX_VELOCITY{80.0};

  static constexpr double MAX_ANGULAR_VELOCITY{M_PI * 4.0};

  control::Point front_match_load_lineup{117.0, 20.0, M_PI / 2.0};

  control::Point front_match_load{117.0, 10.0, M_PI / 2.0};

  control::Point front_wall_blocks_lineup{120.0, 25.0, 0.0};

  control::Point front_wall_blocks{132.0, 25.0, 0.0};

  control::Point far_goal_lineup_1{128.0, 104.0, -M_PI / 2.0};

  control::Point back_matchload_lineup{119.0, 110.0, -M_PI / 2.0};

  control::Point back_matchload{121.0, 129.0, -M_PI / 2.0};

  control::Point back_long_goal{119.0, 98.5, -M_PI / 2.0};

  control::Point park_zone_blocks_lineup{73.75, 100.0, M_PI / 2.0};

  control::Point park_zone_blocks{73.75, 118.0, M_PI / 2.0};

  control::Point wall_blocks_lineup{100.0, 114.0, 0.0};

  control::Point wall_blocks{134.0, 114.0, 0.0};

  control::Point middle_reds_lineup{134.0, 67.0, -M_PI / 2.0};

  control::Point middle_reds{128.0, 64.0, M_PI};

  control::Point middle_goal_lineup_1{134.0, 32.0, -M_PI / 2.0};

  control::Point middle_goal_lineup_2{90.0, 32.0, M_PI};

  control::Point middle_goal{84.0, 58.0, M_PI * 3.0 / 4.0};

  control::Point middle_goal_top_lineup{99.0, 92.0, M_PI / 2.0};

  control::Point middle_goal_top{84.0, 84.0, -M_PI * 3.0 / 4.0};

  control::Point park_lineup_1{134.0, 24.0, -M_PI / 2.0};

  control::Point park_lineup_2{92.0, 12.0, 0.0};

  control::Point park{66.0, 6.0, 0.0};

 public:
  /// @brief Constructs a new orange skills auton object
  OrangeSkillsAuton();

  /// @brief Initializes the auton
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot being controlled
  /// @param control_system __std::shared_ptr<control::ControlSystem>&__ The
  /// control system used
  /// @param process_system __std::shared_ptr<processes::ProcessSystem>&__ The
  /// process system used
  void init(std::shared_ptr<driftless::robot::Robot>& robot,
            std::shared_ptr<driftless::control::ControlSystem>& control_system)
      override;

  /// @brief Runs the auton
  /// @param robot __std::shared_ptr<robot::Robot>&__ The robot being controlled
  /// @param control_system __std::shared_ptr<control::ControlSystem>&__ The
  /// control system used
  /// @param process_system __std::shared_ptr<processes::ProcessSystem>&__ The
  /// process system used
  /// @param alliance __std::shared_ptr<alliance::IAlliance>&__ The current
  /// alliance
  /// @param clock __std::shared_ptr<rtos::IClock>&__ The system clock
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer used
  void run(std::shared_ptr<driftless::robot::Robot>& robot,
           std::shared_ptr<driftless::control::ControlSystem>& control_system,
           std::shared_ptr<driftless::alliance::IAlliance>& alliance,
           std::shared_ptr<rtos::IClock>& clock,
           std::unique_ptr<rtos::IDelayer>& delayer) override;
};
}  // namespace auton
}  // namespace driftless
#endif