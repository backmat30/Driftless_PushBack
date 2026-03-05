#ifndef __BLUE_FCU_AUTON_HPP__
#define __BLUE_FCU_POINT_AUTON_HPP__

#include "driftless/auton/AAuton.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for autonomous routines
/// @author Matthew Backman
namespace auton {

/// @brief Class representing the autonomous routine for the blue robot's
/// anti-UCF auton
/// @author Ethan Jacobson
class BlueFCUAuton : public AAuton {
 private:
  static constexpr double MAX_VELOCITY{80.0};

  static constexpr double MAX_ANGULAR_VELOCITY{M_PI * 2.5};

  // matchload points
  control::Point matchload_lineup{26.0, 20.0, M_PI / 2.0};

  control::Point matchload_lineup_2{27.5, 22.0, M_PI / 2.0};

  control::Point matchload{26.0, 11.0, M_PI / 2.0};

  control::Point matchload_2{29.0, 11.5, M_PI / 2.0};

  // long goal points
  control::Point long_goal{27.25, 41.0, M_PI / 2.0};

  // descore points
  control::Point descore_lineup{15.5, 37.0, M_PI / 2.0};

  control::Point end_descore{16.0, 58.0, M_PI / 2.0};

  // two blocks points
  control::Point two_blocks_lineup{12.0, 58.0, M_PI / 2.0};

  control::Point two_blocks_lineup_2{23.0, 60.5, M_PI / 4.0};

  control::Point two_blocks{29.0, 62.5, M_PI / 2.0};

  // line block points
  control::Point line_blocks_lineup{30.0, 62.5, M_PI / 4.0};

  control::Point line_blocks_lineup_2{31.5, 66.25, M_PI / 4.0};

  control::Point line_blocks{51.0, 66.25, M_PI / 4.0};

  // middle goal points
  control::Point bottom_goal_lineup{55.0, 52.0, M_PI / 2.0};

  control::Point bottom_goal_lineup_2{90.5, 52.0, M_PI / 2.0};

  control::Point bottom_goal{71.5, 76.0, M_PI * 2.0 / 3.0};

  control::Point top_goal_lineup{51, 48, M_PI / 4};

  control::Point top_goal{63.25, 58.75, M_PI / 4};

  // other side descore
  control::Point descore_2_lineup{50.0, 50.0, M_PI / 4.0};

  control::Point descore_2{38.5, 38.0, M_PI / 2};

  control::Point descore_2_end{39.25, 45.0, M_PI / 2};

 public:
  /// @brief Constructs a new blue fcu auton object
  BlueFCUAuton();

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