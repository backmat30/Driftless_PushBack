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

  control::Point front_match_load_lineup{116.0, 24.0, M_PI / 2.0};

  control::Point front_match_load{116.0, 11.0, M_PI / 2.0};

  control::Point front_wall_blocks_lineup{120.0, 22.0, 0.0};

  control::Point front_wall_blocks{132.0, 24.0, 0.0};

  control::Point far_goal_lineup_1{132.0, 100.0, -M_PI / 2.0};

  control::Point back_matchload_lineup{119.0, 110.0, -M_PI / 2.0};

  control::Point back_matchload{120.0, 129.0, -M_PI / 2.0};

  control::Point back_long_goal{119.0, 98.0, -M_PI / 2.0};

  control::Point park_zone_blocks_lineup{74.0, 100.0, M_PI / 2.0};

  control::Point park_zone_blocks{73.0, 118.0, M_PI / 2.0};

  control::Point wall_blocks_lineup{100.0, 114.0, 0.0};

  control::Point wall_blocks_start{134.0, 114.0, 0.0};

  control::Point wall_blocks_end{136.0, 74.0, -M_PI / 2.0};

  control::Point middle_reds{109.0, 54.0, -M_PI * 3.0 / 4.0};

  control::Point middle_reds_end{110.0, 48.0, -M_PI / 2.0};

  control::Point middle_goal_lineup{90.0, 48.0, M_PI * 3.0 / 4.0};

  control::Point middle_goal{84.5, 57.5, M_PI * 3.0 / 4.0};

  control::Point middle_goal_top_lineup{99.0, 92.0, M_PI / 2.0};

  control::Point middle_goal_top{84.0, 84.0, -M_PI * 3.0 / 4.0};

  control::Point park_lineup_1{134.0, 24.0, -M_PI / 2.0};

  control::Point park_lineup_2{92.0, 12.0, 0.0};

  control::Point park{72.0, 10.0, 0.0};

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