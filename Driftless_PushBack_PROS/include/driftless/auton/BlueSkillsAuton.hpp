#ifndef __BLUE_SKILLS_AUTON_HPP__
#define __BLUE_SKILLS_AUTON_HPP__

#include "driftless/auton/AAuton.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for autonomous routines
/// @author Matthew Backman
namespace auton {

/// @brief Class representing the autonomous routine for the blue robot's skills
/// route
/// @author Matthew Backman
class BlueSkillsAuton : public AAuton {
 private:
  static constexpr double MAX_VELOCITY{80.0};

  static constexpr double MAX_ANGULAR_VELOCITY{M_PI * 4.0};

  control::Point orange_preload{80.0, 27.0, 0.0};

  control::Point first_matchload_lineup{25.0, 28.0, M_PI / 2};

  control::Point first_matchload{25.25, 10.5, M_PI / 2.0};

  control::Point left_goal_front{26.0, 42.0, M_PI / 2.0};

  control::Point park_zone_blocks_lineup{73.0, 42.0, -M_PI / 2.0};

  control::Point park_zone_blocks{73.0, 24.0, -M_PI / 2.0};

  control::Point park_zone_grab{73.0, 26.0, -M_PI / 2.0};

  control::Point front_left_wall_blocks_lineup{26.0, 24.0, M_PI};

  control::Point front_left_wall_blocks{10.0, 26.0, M_PI};

  control::Point middle_blocks_lineup{12.0, 68.0, M_PI / 2.0};

  control::Point middle_blocks_lineup_2{12.0, 84.0, 0.0};

  control::Point middle_blocks_end{32.0, 84.0, 0.0};

  control::Point middle_top_goal_lineup{52.0, 57.5, M_PI / 4.0};

  control::Point middle_top_goal{60.5, 59.5, M_PI / 4.0};

  control::Point middle_lower_goal_lineup{50.0, 84.0, M_PI / 4.0};

  control::Point middle_lower_goal{61.0, 87.0, M_PI / 4.0};

  control::Point back_left_wall_blocks_lineup_1{42.0, 74.0, M_PI / 4.0};

  control::Point back_left_wall_blocks_lineup_2{42.0, 120.0, M_PI};

  control::Point back_left_wall_blocks{9.0, 121.0, M_PI};

  control::Point second_matchload_lineup{26.0, 118.0, -M_PI / 2.0};

  control::Point second_matchload{26.0, 138.0, -M_PI / 2.0};

  control::Point left_goal_back{26.0, 103.0, -M_PI / 2.0};

  control::Point park_lineup_1{12.0, 106.0, M_PI};

  control::Point park_lineup_2{12.0, 20.0, M_PI};

  control::Point park_zone{80.0, 9.0, M_PI};

 public:
  /// @brief Constructs a new blue skills auton object
  BlueSkillsAuton();

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