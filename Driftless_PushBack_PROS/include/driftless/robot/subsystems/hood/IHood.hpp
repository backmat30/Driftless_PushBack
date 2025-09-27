#ifndef __I_HOOD_HPP__
#define __I_HOOD_HPP__

/// @brief The namespace for driftless library code
/// @author Matthew Backman
namespace driftless {

/// @brief The namespace for robot code
/// @author Matthew Backman
namespace robot {

/// @brief The namespace for subsystems code
/// @author Matthew Backman
namespace subsystems {

/// @brief Namespace for the hood subsystem
/// @author Matthew Backman
namespace hood {

/// @brief Interface for a generic hood object
/// @author Matthew Backman
class IHood {
 public:
  /// @brief Destroys the hood object
  virtual ~IHood() = default;

  /// @brief Initializes the hood
  virtual void init() = 0;

  /// @brief Runs the hood
  virtual void run() = 0;

  /// @brief Sets the voltage of the hood motors
  /// @param voltage __double__ The voltage
  virtual void setVoltage(double voltage) = 0;

  /// @brief Opens the gate at the end of the hood
  virtual void open() = 0;

  /// @brief Closes the gate at the end of the hood
  virtual void close() = 0;

  /// @brief Toggles the gate between open and closed
  virtual void toggleOpen() = 0;

  /// @brief Raises the hood
  virtual void raise() = 0;

  /// @brief Lowers the hood
  virtual void lower() = 0;

  /// @brief Toggles the hood between raised and lowered
  virtual void toggleRaised() = 0;

  /// @brief Checks if the hood is raised
  /// @return __bool__ True if the hood is raised, false otherwise
  virtual bool isRaised() = 0;
};
}  // namespace hood
}  // namespace subsystems
}  // namespace robot
}  // namespace driftless
#endif