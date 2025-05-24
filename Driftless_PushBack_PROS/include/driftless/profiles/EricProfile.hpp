#ifndef __ERIC_PROFILE_HPP__
#define __ERIC_PROFILE_HPP__

#include <map>
#include <string>

#include "driftless/op_control/EControl.hpp"
#include "driftless/op_control/EControlType.hpp"
#include "driftless/op_control/EControllerAnalog.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/profiles/IProfile.hpp"

/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief Namespace for user control profiles
/// @author Matthew Backman
namespace profiles {

/// @brief User control profile for Eric
/// @author Matthew Backman
class EricProfile : public driftless::profiles::IProfile {
private:
  /// @brief Name of the profile
  static constexpr char PROFILE_NAME[]{"ERIC"};

  /// @brief List of subsystems to be controlled and the type of control used
  std::map<op_control::EControlType, int> CONTROL_MODE_MAP{};

  /// @brief Maps subsystem controls to analog inputs
  const std::map<op_control::EControl, op_control::EControllerAnalog>
      ANALOG_CONTROL_MAP{};

  /// @brief Maps subsystem controls to digital inputs
  const std::map<op_control::EControl, op_control::EControllerDigital>
      DIGITAL_CONTROL_MAP{};

  const std::map<op_control::EStartupConfig, bool> STARTUP_CONFIG_MAP{};

public:
  /// @brief Gets the name of the profile
  /// @return __std::string__ The name
  std::string getName() override;

  /// @brief Gets the control mode of the given control type
  /// @param control_type __op_control::EControlType__ The control type
  /// @return __int__ The control mode as an integer
  int getControlMode(op_control::EControlType control_type) const override;

  /// @brief Sets the control mode of the given control type
  /// @param control_type __op_control::EControlType__ The control type
  /// @param control_mode __int__ The new control mode
  void setControlMode(op_control::EControlType control_type,
                      int control_mode) override;

  /// @brief Gets the analog control mapped to the given control
  /// @param control __op_control::EControl__ The control
  /// @return __op_control::EControllerAnalog__ The analog control mapped to the
  /// control
  op_control::EControllerAnalog
  getAnalogControlMapping(op_control::EControl control) const override;

  /// @brief Gets the digital control mapped to the given control
  /// @param control __op_control::EControl__ The control
  /// @return __op_control::EControllerDigital__ The digital control mapped to
  /// the control
  op_control::EControllerDigital
  getDigitalControlMapping(op_control::EControl control) const override;

  /// @brief Gets the value of the desired startup configuration
  /// @param startup_config __op_control::EStartupConfig__ The desired startup
  /// configuration
  /// @return __bool__ The value of the desired startup configuration
  bool
  getStartupConfig(op_control::EStartupConfig startup_config) const override;
};
} // namespace profiles
} // namespace driftless
#endif