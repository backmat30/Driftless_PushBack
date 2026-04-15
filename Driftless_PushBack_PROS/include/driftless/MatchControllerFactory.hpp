#ifndef __MATCH_CONTROLLER_FACTORY_HPP__
#define __MATCH_CONTROLLER_FACTORY_HPP__

// general includes
#include "driftless/MatchController.hpp"
#include "driftless/menu/MenuAdapter.hpp"
#include "driftless/pros_adapters/ProsClock.hpp"
#include "driftless/pros_adapters/ProsDelayer.hpp"

// alliance includes
#include "driftless/alliance/BlueAlliance.hpp"
#include "driftless/alliance/RedAlliance.hpp"
#include "driftless/alliance/SkillsAlliance.hpp"

// auton includes
#include "driftless/auton/BlueControlRushAuton.hpp"
#include "driftless/auton/BlueControlRushLeft.hpp"
#include "driftless/auton/BlueFCUAuton.hpp"
#include "driftless/auton/BluePumpNDumpAuton.hpp"
#include "driftless/auton/BlueSkillsAuton.hpp"
#include "driftless/auton/BlueWPAuton.hpp"
#include "driftless/auton/FeedForwardTunerAuton.hpp"
#include "driftless/auton/OrangeControlRushAuton.hpp"
#include "driftless/auton/OrangePumpNDumpAuton.hpp"
#include "driftless/auton/OrangeSkillsAuton.hpp"
#include "driftless/auton/OrangeWPAuton.hpp"

// config includes
#include "driftless/config/BlueConfig.hpp"
#include "driftless/config/OrangeConfig.hpp"

// profile includes
#include "driftless/profiles/AsherProfile.hpp"
#include "driftless/profiles/EthanProfile.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {
/// @brief Factory for creating a MatchController object
/// @author Matthew Backman
class MatchControllerFactory {
 public:
  /// @brief Creates a new MatchController object
  /// @return __driftless::MatchController__ The new MatchController object
  static driftless::MatchController createMatchController();
};
}  // namespace driftless
#endif