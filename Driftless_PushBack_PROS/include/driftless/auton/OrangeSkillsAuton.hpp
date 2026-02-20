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

  control::Point middle_blues_lineup_1{130.0, 36.0, M_PI};

  control::Point middle_blues_lineup_2{128.0, 79.0, M_PI};

  control::Point middle_blues{118.0, 78.5, M_PI};

  control::Point back_matchload_lineup{96.0, 95.0, M_PI};

  control::Point back_matchload_lineup_2{118.0, 116.0, M_PI * 3.0 / 4.0};

  control::Point back_matchload{118.0, 134.0, -M_PI / 2.0};

  control::Point back_long_goal{121.0, 96.0, -M_PI / 2.0};

  control::Point park_zone_blocks_lineup{70.0, 94.0, M_PI / 2.0};

  control::Point park_zone_blocks{70.0, 121.0, M_PI / 2.0};

  control::Point wall_blocks_lineup{100.0, 120.0, 0.0};

  control::Point wall_blocks_start{134.0, 128.0, -M_PI / 4.0};

  control::Point wall_blocks_end{134.0, 64.0, -M_PI / 4.0};

  control::Point middle_reds{118.0, 64.0, M_PI};

  control::Point middle_goal_lineup{96.0, 48.0, M_PI * 3.0 / 4.0};

  control::Point middle_goal{83.0, 59.0, M_PI * 3.0 / 4.0};

  control::Point front_wall_blocks_lineup{90.0, 28.0, M_PI * 3.0 / 4.0};

  control::Point front_wall_blocks_start{132.0, 12.0, M_PI / 4.0};

  control::Point front_wall_blocks_end{134.0, 50.0, M_PI / 4.0};

  control::Point front_match_load_lineup{118.0, 36.0, M_PI / 2.0};

  control::Point front_match_load{118.0, 10.0, M_PI / 2.0};

  control::Point front_right_long_goal{118.0, 44.0, M_PI / 2.0};

  control::Point park_lineup{92.0, 12.0, 0.0};

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