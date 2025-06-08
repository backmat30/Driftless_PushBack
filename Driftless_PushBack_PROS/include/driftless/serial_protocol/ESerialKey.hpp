#ifndef __E_SERIAL_KEY_HPP__
#define __E_SERIAL_KEY_HPP__
#include <cstdint>
/// @brief Namespace for driftless library code
/// @author Matthew Backman
namespace driftless {
  /// @brief Namespace for serial communication protocols
  /// @author Matthew Backman
  namespace serial_protocol {
    /// @brief Enumeration representing valid keys for serial communication
    /// @author Matthew Backman
    enum class ESerialKey : int8_t {
      XPOS = 'X',
      YPOS = 'Y',
      THETA = 'H',
      SWERVE_MODULE_A = 'A',
      SWERVE_MODULE_B = 'B',
      SWERVE_MODULE_C = 'C',
      SWERVE_MODULE_D = 'D'
    };
  }
}
#endif