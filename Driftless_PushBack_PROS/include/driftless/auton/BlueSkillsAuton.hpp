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

  control::Point orange_preload{80.0, 26.0, 0.0};

  control::Point first_matchload_lineup{27.0, 32.0, M_PI / 2};

  control::Point first_matchload{27.5, 10.5, M_PI / 2.0};

  control::Point left_goal_front{26.5, 41.5, M_PI / 2.0};

  control::Point park_zone_blocks_lineup{73.5, 42.0, -M_PI / 2.0};

  control::Point park_zone_blocks{73.5, 23.0, -M_PI / 2.0};

  control::Point front_left_wall_blocks_lineup{26.0, 24.0, M_PI};

  control::Point front_left_wall_blocks{11.0, 23.0, M_PI};

  control::Point middle_blocks_lineup{12.0, 36.0, M_PI / 2.0};

  control::Point middle_blocks_lineup_2{12.0, 66.0, 0.0};

  control::Point middle_blocks_end{32.0, 78.0, M_PI / 6.0};

  control::Point middle_top_goal_lineup{54.0, 50.0, M_PI / 4.0};

  control::Point middle_top_goal{59.0, 53.0, M_PI / 4.0};

  control::Point middle_lower_goal_lineup{50.0, 88.0, M_PI / 4.0};

  control::Point middle_lower_goal{59.0, 81.0, M_PI / 4.0};

  control::Point back_left_wall_blocks_lineup_1{50.0, 74.0, M_PI / 4.0};

  control::Point back_left_wall_blocks_lineup_2{50.0, 114.0, M_PI};

  control::Point back_left_wall_blocks{9.0, 113.0, M_PI};

  control::Point second_matchload_lineup{27.0, 118.0, -M_PI / 2.0};

  control::Point second_matchload{28.0, 132.0, -M_PI / 2.0};

  control::Point left_goal_back{28.0, 97.0, -M_PI / 2.0};

  control::Point park_lineup_1{12.0, 106.0, M_PI};

  control::Point park_lineup_2{10.0, 32.0, M_PI};

  control::Point park_lineup_3{42.0, 10.0, M_PI};

  control::Point park_zone{67.0, 8.0, M_PI};

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