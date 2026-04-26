#ifndef __ORANGE_MAX_POINT_AUTON_HPP__
#define __ORANGE_MAX_POINT_AUTON_HPP__

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
class OrangeMaxPointAuton : public AAuton {
 private:
  static constexpr double MAX_VELOCITY{80.0};

  static constexpr double MAX_ANGULAR_VELOCITY{M_PI * 4.0};

  // matchload points
  control::Point match_load_lineup{117.0, 24.0, M_PI / 2.0};

  control::Point match_load_lineup_2{117.0, 28.0, M_PI / 2.0};

  control::Point match_load{117.0, 11.0, M_PI / 2.0};

  // long goal points
  control::Point long_goal{116.0, 42.0, M_PI / 2.0};

  // descore points
  control::Point descore_lineup{127.0, 37.0, M_PI / 2.0};

  control::Point start_descore{126.0, 48.0, M_PI / 2.0};

  control::Point end_descore{126.0, 60.0, M_PI / 2.0};

  // blocks points
  control::Point two_blocks_lineup{132.0, 56.0, M_PI};

  control::Point two_blocks_lineup_2{123, 57.0, M_PI * 3.0 / 4.0};

  control::Point two_blocks{120.0, 59.0, M_PI / 2.0};

  control::Point line_blocks_lineup{120.0, 60.0, M_PI * 3.0 / 4.0};

  control::Point line_blocks_lineup_2{115.5, 63.0, M_PI * 3.0 / 4.0};

  control::Point line_blocks{103.0, 67.0, M_PI};

  // middle goal points
  control::Point bottom_goal_lineup{93.0, 54.0, M_PI * 3.0 / 4.0};

  control::Point bottom_goal{83.0, 57.0, M_PI * 3.0 / 4.0};

 public:
  /// @brief Constructs a new orange win point auton object
  OrangeMaxPointAuton();

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