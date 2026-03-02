#ifndef __BLUE_CONTROL_RUSH_AUTON_HPP__
#define __BLUE_CONTROL_RUSH_AUTON_HPP__

#include "driftless/auton/AAuton.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for autonomous routines
/// @author Matthew Backman
namespace auton {

/// @brief Class representing the autonomous routine for the blue robot's win point auton
/// @author Ethan jacobson
class BlueControlRushAuton : public AAuton {
 private:
  static constexpr double MAX_VELOCITY{80.0};

  // matchload points
  control::Point matchload_lineup{26.0, 22.0, M_PI / 2.0};

  control::Point matchload_lineup_2{29.0, 22.0, M_PI / 2.0};

  control::Point matchload{26.0, 11.0, M_PI / 2.0};

  control::Point matchload_2{28.0, 11.5, M_PI / 2.0};

  // long goal points
  control::Point long_goal{27.5, 42.0, M_PI / 2.0};

  // descore points
  control::Point descore_lineup{38.0, 37.0, M_PI / 2.0};
  
  control::Point end_descore{37.0, 60.5, M_PI / 2.0};

 public:
  /// @brief Constructs a new blue pump n dump auton object
  BlueControlRushAuton();

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