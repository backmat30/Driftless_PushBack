#ifndef __BLUE_CENTER_AUTON_HPP__
#define __BLUE_CENTER_AUTON_HPP__

#include "driftless/auton/AAuton.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for autonomous routines
/// @author Matthew Backman
namespace auton {

/// @brief Class representing the autonomous routine for the blue robot's center match auton
/// @author Ethan jacobson
class BlueCenterAuton : public AAuton {
 private:
  static constexpr double MAX_VELOCITY{80.0};

  control::Point matchload_lineup{26.0, 26.0, M_PI / 2.0};

  control::Point matchload{26, 11.5, M_PI / 2.0};

  control::Point matchload_2{27, 11.5, M_PI / 2.0};

  control::Point long_goal_lineup{28.0, 26.0, M_PI / 4.0};

  control::Point long_goal_lineup_2{28.0, 31.0, M_PI / 4.0};

  control::Point long_goal{26, 42.0, M_PI / 2.0};

  control::Point long_goal_to_matchload{12.0, 12.0, M_PI / 2.0};

  control::Point descore_lineup{14.0, 38.0, M_PI / 2.0};

  control::Point start_descore{16.0, 45.0, M_PI / 2.0};
  
  control::Point end_descore{16.0, 58.0, M_PI / 2.0};

  control::Point center_two_blocks_lineup{12.0, 50.0, M_PI / 5.5};

  control::Point center_two_blocks{21.0, 61.0, M_PI / 4.0};

  control::Point center_two_blocks_2{24, 62.0, M_PI / 2.5};

  control::Point center_two_blocks_3{26.0, 64.0, M_PI / 2};

  control::Point center_six_blocks_lineup{30.0, 62.0, M_PI / 2};

  control::Point center_six_blocks{32.0, 67.5, M_PI / 4};

  control::Point center_six_blocks_2{41.5, 70, M_PI / 6};

  control::Point center_six_blocks_3{51.5, 72.0, M_PI / 5};

  control::Point bottom_goal_lineup{54.0, 78.0, 0.0};

  control::Point bottom_goal{58.0, 83.0, 0};

  control::Point bottom_goal_1{58.0, 85.0, 0};

  control::Point bottom_goal_2{76, 76, - M_PI / 8};

  control::Point matchload_lineup_2{38, 38.0, M_PI / 2};

  control::Point top_goal_lineup{51, 48, M_PI / 4};

  control::Point top_goal{61.5, 58.5, M_PI / 4};

//   control::Point park_lineup{44, 20, 0};

//   control::Point park_lineup_2{46.0, 10.0, 0.0};

//   control::Point park_zone{80.0, 6.0, 0.0};

  control::Point park_lineup{52.0, 12.0, 0};

  control::Point park_lineup_2{45.0, 10.0, 0.0};

  control::Point park_zone{80.0, 5.0, 0.0};


 public:
  /// @brief Constructs a new blue pump n dump auton object
  BlueCenterAuton();

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