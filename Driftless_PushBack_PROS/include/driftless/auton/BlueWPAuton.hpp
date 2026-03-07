#ifndef __BLUE_WIN_POINT_AUTON_HPP__
#define __BLUE_WIN_POINT_AUTON_HPP__

#include "driftless/auton/AAuton.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for autonomous routines
/// @author Matthew Backman
namespace auton {

/// @brief Class representing the autonomous routine for the blue robot's win
/// point auton
/// @author Ethan jacobson
class BlueWPAuton : public AAuton {
 private:
  static constexpr double MAX_VELOCITY{80.0};

  static constexpr double MAX_ANGULAR_VELOCITY{M_PI * 4.0};

  // matchload points
  control::Point matchload_lineup{28.0, 22.0, M_PI / 2.0};

  control::Point matchload_lineup_2{29.5, 22.0, M_PI / 2.0};

  control::Point matchload{27.5, 11.0, M_PI / 2.0};

  control::Point matchload_2{28.5, 11.5, M_PI / 2.0};

  // long goal points
  control::Point long_goal{26.75, 42.0, M_PI / 2.0};

  // descore points
  control::Point descore_lineup{16.0, 37.0, M_PI / 2.0};

  control::Point end_descore{16.5, 58.0, M_PI / 2.0};

  // two blocks points
  control::Point two_blocks_lineup{12.0, 56.0, 0.0};

  control::Point two_blocks_lineup_2{23.0, 59.0, M_PI / 4.0};

  control::Point two_blocks{29.0, 62.0, M_PI / 2.0};

  // line block points
  control::Point line_blocks_lineup{30.0, 62.5, M_PI / 4.0};

  control::Point line_blocks_lineup_2{32.5, 67, M_PI / 4.0};

  control::Point line_blocks{49.0, 66.0, M_PI / 4.0};

  // middle goal points
  control::Point bottom_goal_1{60.0, 81.0, 0};

  control::Point bottom_goal_2{77, 75, -M_PI / 5};

  control::Point top_goal_lineup{51, 48, M_PI / 4};

  control::Point top_goal{62.5, 58.5, M_PI / 4};

  // other side descore
  control::Point descore_2_lineup{50.0, 50.0, M_PI / 4.0};

  control::Point descore_2{38.5, 38.0, M_PI / 2};

  control::Point descore_2_end{38.0, 61.0, M_PI / 2};

 public:
  /// @brief Constructs a new blue pump n dump auton object
  BlueWPAuton();

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