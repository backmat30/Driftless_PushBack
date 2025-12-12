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

  control::Point orange_preload{80.0, 25.0, 0.0};

  control::Point first_matchload_lineup{32.0, 28.0, M_PI / 2};

  control::Point first_matchload{26.0, 11.0, M_PI / 2.0};

  control::Point left_goal_front{26.0, 42.0, M_PI / 2.0};

  control::Point front_left_wall_blocks_lineup_1{26.0, 32.0, M_PI / 2.0};

  control::Point front_left_wall_blocks_lineup_2{10.0, 40.0,
                                                 -M_PI * 145.0 / 180.0};

  control::Point front_left_wall_blocks_mid{10.0, 22.0, -M_PI * 130.0 / 180.0};

  control::Point front_left_wall_blocks_end{6.0, 8.0, -M_PI * 140.0 / 180.0};

  control::Point park_zone_blocks_lineup{73.0, 44.0, -M_PI / 2.0};

  control::Point park_zone_blocks{73.0, 24.0, -M_PI / 2.0};

  control::Point park_zone_grab{73.0, 26.0, -M_PI / 2.0};

  control::Point middle_blocks_lineup{42.0, 56.0, 3 * M_PI / 4.0};

  control::Point middle_blocks_lineup_2{44.0, 92.0, M_PI};

  control::Point middle_blocks_end{20.0, 72.0, -3.0 * M_PI / 4.0};

  control::Point middle_top_goal_lineup{54.0, 52.0, M_PI / 4.0};

  control::Point middle_top_goal{59.5, 60.0, M_PI / 4.0};

  control::Point middle_lower_goal_lineup{50.0, 84.0, M_PI / 4.0};

  control::Point middle_lower_goal{66.0, 84.0, M_PI / 4.0};

  control::Point back_left_wall_blocks_lineup{10.0, 74.0, M_PI * 130.0 / 180.0};

  control::Point back_left_wall_blocks_start{10.0, 114.0, M_PI * 130.0 / 180.0};

  control::Point back_left_wall_blocks_end{10.0, 140.0, M_PI * 130.0 / 180.0};

  control::Point second_matchload_lineup{26.0, 118.0, -M_PI / 2.0};

  control::Point second_matchload{25.0, 138.0, -M_PI / 2.0};

  control::Point left_goal_back{24.0, 102.0, -M_PI / 2.0};

  control::Point park_lineup_1{48.0, 60.0, 0.0};

  control::Point park_lineup_2{38.0, 12.0, 0.0};

  control::Point park_zone{80.0, 6.0, 0.0};

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