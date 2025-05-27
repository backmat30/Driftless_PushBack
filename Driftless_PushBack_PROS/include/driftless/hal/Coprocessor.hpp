#ifndef __COPROCESSOR_HPP__
#define __COPROCESSOR_HPP__

#include <map>
#include <memory>
#include <string>

#include "driftless/io/ISerialDevice.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"

namespace driftless {
namespace hal {
class Coprocessor {
 private:
  static constexpr uint8_t TASK_DELAY{10};

  static void taskLoop(void* params);

  std::unique_ptr<io::ISerialDevice> m_serial_device;

  std::unique_ptr<rtos::ITask> m_task;

  std::unique_ptr<rtos::IClock> m_clock;

  std::unique_ptr<rtos::IDelayer> m_delayer;

  std::unique_ptr<rtos::IMutex> m_mutex;

  std::map<char, std::string> m_latest_data;

  std::string m_serial_buffer;

  void TaskUpdate();

  void fetchLatestSignal();

  public:
  /// @brief Constructs a new Coprocessor object
  /// @param serialDevice __std::unique_ptr<io::ISerialDevice>&__ The serial device used for the coprocessor
  Coprocessor(std::unique_ptr<io::ISerialDevice>& serialDevice);

  /// @brief Initializes the Coprocessor object
  void init();

  /// @brief Gets the latest value of a given key, if available
  /// @param key __char__ The key value to search for
  /// @return __std::string__ The value associated with the key, or an empty string if not found
  std::string getValue(char key);

  /// @brief Sends a key-value pair to the coprocessor
  /// @param key __char__ The key to send
  /// @param value __std::string&__ The value to send
  void sendValue(char key, const std::string& value);
};
}  // namespace hal
}  // namespace driftless
#endif