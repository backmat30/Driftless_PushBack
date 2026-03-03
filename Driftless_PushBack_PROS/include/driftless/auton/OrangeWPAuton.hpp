#ifndef __ORANGE_WIN_POINT_AUTON_HPP__
#define __ORANGE_WIN_POINT_AUTON_HPP__

#include "driftless/auton/AAuton.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for autonomous routines
/// @author Matthew Backman
namespace auton {

/// @brief Class representing the autonomous routine for the orange robot's
/// win point auton routine
/// @author Ethan Jacobson
class OrangeWPAuton : public AAuton {
 private:
  static constexpr double MAX_VELOCITY{80.0};

  static constexpr double MAX_ANGULAR_VELOCITY{M_PI * 2.5};

  // matchload points
  control::Point matchload_lineup{120.0, 22.0, M_PI / 2.0};

  control::Point matchload{120.0, 11.0, M_PI / 2.0};

  // long goal points
  control::Point long_goal_lineup{121.0, 20.0, M_PI * 3.0 / 4.0};

  control::Point long_goal_lineup_2{121.0, 33.0, M_PI * 3.0 / 4.0};

  control::Point long_goal{120.0, 42.0, M_PI / 2.0};

  // descore points
  control::Point descore_lineup{131.0, 37.0, M_PI / 2.0};

  control::Point start_descore{130.0, 48.0, M_PI / 2.0};

  control::Point end_descore{130.5, 60.0, M_PI / 2.0};

  // blocks points
  control::Point two_blocks_lineup{136.0, 56.0, M_PI};

  control::Point two_blocks_lineup_2{126, 59, M_PI * 3.0 / 4.0};

  control::Point two_blocks{123.0, 62.0, M_PI / 2.0};

  control::Point line_blocks_lineup{120.0, 62.5, M_PI * 3.0 / 4.0};

  control::Point line_blocks_lineup_2{116.0, 67.0, M_PI * 3.0 / 4.0};

  control::Point line_blocks{99.0, 66.0, M_PI * 3.0 / 4.0};

  // middle goal points
  control::Point top_goal_lineup{93.0, 88.0, M_PI * 5.0 / 4.0};

  control::Point top_goal{88.75, 86, M_PI * 5.0 / 4.0};

  control::Point bottom_goal{84, 61.0, M_PI * 3.0 / 4.0};

  // other side descore
  control::Point descore_2_lineup{94.0, 50.0, M_PI * 3.0 / 4.0};

  control::Point descore_2{108.5, 38.0, M_PI / 2};

  control::Point descore_2_end{109.0, 61.0, M_PI / 2};

 public:
  /// @brief Constructs a new orange win point auton object
  OrangeWPAuton();

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