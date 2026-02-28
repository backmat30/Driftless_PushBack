#ifndef __ORANGE_PUMP_N_DUMP_AUTON_HPP__
#define __ORANGE_PUMP_N_DUMP_AUTON_HPP__

#include "driftless/auton/AAuton.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for autonomous routines
/// @author Matthew Backman
namespace auton {

/// @brief Class representing the autonomous routine for the orange robot's pump
/// and dump match auton
/// @author Matthew Backman
class OrangePumpNDumpAuton : public AAuton {
 private:
  static constexpr double MAX_VELOCITY{80.0};

  control::Point matchload_lineup{120.0, 28.0, M_PI / 2.0};

  control::Point matchload{120.0, 11.0, M_PI / 2.0};

  control::Point long_goal_lineup{121.0, 20.0, M_PI * 3.0 / 4.0};

  control::Point long_goal_lineup_2{121.0, 33.0, M_PI * 3.0 / 4.0};

  control::Point long_goal{120.0, 42.0, M_PI / 2.0};

  control::Point long_goal_2{120.0, 42.0, M_PI / 2.0};

  control::Point long_goal_to_matchload{118.0, 12.0, M_PI / 2.0};

  control::Point descore_lineup{131.0, 33.0, M_PI / 2.0};

  control::Point start_descore{130.0, 48.0, M_PI / 2.0};

  control::Point end_descore{130.5, 60.0, M_PI / 2.0};

 public:
  /// @brief Constructs a new orange pump n dump auton object
  OrangePumpNDumpAuton();

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