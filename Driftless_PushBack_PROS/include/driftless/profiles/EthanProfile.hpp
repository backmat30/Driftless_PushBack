#ifndef __ETHAN_PROFILE_HPP__
#define __ETHAN_PROFILE_HPP__

#include <map>
#include <string>

#include "driftless/op_control/EControl.hpp"
#include "driftless/op_control/EControlType.hpp"
#include "driftless/op_control/EControllerAnalog.hpp"
#include "driftless/op_control/EControllerDigital.hpp"
#include "driftless/op_control/hood/EHoodControlMode.hpp"
#include "driftless/op_control/intake/EIntakeControlMode.hpp"
#include "driftless/profiles/IProfile.hpp"

namespace driftless {
namespace profiles {
class EthanProfile : public IProfile {
 private:
  /// @brief Name of the profile
  static constexpr char PROFILE_NAME[]{"ETHAN"};

  /// @brief Map of control modes for each subsystem
  std::map<op_control::EControlType, int> CONTROL_MODE_MAP{
      {op_control::EControlType::INTAKE,
       static_cast<int>(op_control::intake::EIntakeControlMode::SMART_SPLIT)},
      {op_control::EControlType::HOOD,
       static_cast<int>(op_control::hood::EHoodControlMode::SMART_TOGGLE)}};

  /// @brief Maps subsystem controls to analog inputs
  const std::map<op_control::EControl, op_control::EControllerAnalog>
      ANALOG_CONTROL_MAP{{op_control::EControl::HOLONOMIC_FORWARD,
                          op_control::EControllerAnalog::JOYSTICK_LEFT_Y},
                         {op_control::EControl::HOLONOMIC_STRAFE,
                          op_control::EControllerAnalog::JOYSTICK_LEFT_X},
                         {op_control::EControl::HOLONOMIC_TURN,
                          op_control::EControllerAnalog::JOYSTICK_RIGHT_X}};

  /// @brief Maps subsystem controls to digital inputs
  const std::map<op_control::EControl, op_control::EControllerDigital>
      DIGITAL_CONTROL_MAP{
          {op_control::EControl::INTAKE_FRONT_RUN_IN,
           op_control::EControllerDigital::TRIGGER_RIGHT_TOP},
          {op_control::EControl::INTAKE_FRONT_RUN_OUT,
           op_control::EControllerDigital::TRIGGER_LEFT_BOTTOM},
          {op_control::EControl::INTAKE_BACK_RUN_IN,
           op_control::EControllerDigital::TRIGGER_RIGHT_BOTTOM},
          {op_control::EControl::HOOD_TOGGLE_RAISED,
           op_control::EControllerDigital::BUTTON_B},
          {op_control::EControl::HOOD_TOGGLE_GATE,
           op_control::EControllerDigital::DPAD_DOWN},
          {op_control::EControl::HOOD_TOGGLE_DESCORE,
           op_control::EControllerDigital::TRIGGER_LEFT_TOP},
          {op_control::EControl::HOOD_TOGGLE_BUMP,
           op_control::EControllerDigital::BUTTON_Y},
          {op_control::EControl::BRAKE_TOGGLE,
           op_control::EControllerDigital::DPAD_RIGHT},
          {op_control::EControl::RAKE_TOGGLE,
           op_control::EControllerDigital::DPAD_UP}};

  /// @brief Maps startup configurations to their values
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
  op_control::EControllerAnalog getAnalogControlMapping(
      op_control::EControl control) const override;

  /// @brief Gets the digital control mapped to the given control
  /// @param control __op_control::EControl__ The control
  /// @return __op_control::EControllerDigital__ The digital control mapped to
  /// the control
  op_control::EControllerDigital getDigitalControlMapping(
      op_control::EControl control) const override;

  /// @brief Gets the value of the desired startup configuration
  /// @param startup_config __op_control::EStartupConfig__ The desired startup
  /// configuration
  /// @return __bool__ The value of the desired startup configuration
  bool getStartupConfig(
      op_control::EStartupConfig startup_config) const override;
};
}  // namespace profiles
}  // namespace driftless

#endif