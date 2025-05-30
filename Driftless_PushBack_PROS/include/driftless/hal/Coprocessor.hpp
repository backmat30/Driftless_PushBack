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
#include "driftless/serial_protocol/ISerialProtocol.hpp"

namespace driftless {
namespace hal {
class Coprocessor {
 private:
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Performs task updates in a constant loop
  /// @param params __void*__ Pointer to the coprocessor to update
  static void taskLoop(void* params);

  std::unique_ptr<io::ISerialDevice> m_serial_device;

  std::unique_ptr<serial_protocol::ISerialProtocol> m_serial_protocol;

  std::unique_ptr<rtos::ITask> m_task;

  std::unique_ptr<rtos::IClock> m_clock;

  std::unique_ptr<rtos::IDelayer> m_delayer;

  std::unique_ptr<rtos::IMutex> m_mutex;

  std::map<std::string, std::string> m_latest_data;

  std::string m_serial_buffer;

  /// @brief Updates the coprocessor object
  void taskUpdate();

  /// @brief Fetches the latest signal from the coprocessor and stores it as
  /// key-value pairs
  void fetchLatestSignal();

  /// @brief finds and stores packets recieved from the latest signal
  void processLatestSignal();

  /// @brief Determines if the buffer contains a complete packet
  /// @return __bool__ True if a complete packet is available, false otherwise
  bool hasPacket() const;

 public:
  /// @brief Initializes the Coprocessor object
  void init();

  /// @brief Gets the latest value of a given key, if available
  /// @param key __std::string&__ The key value to search for
  /// @return __std::string__ The value associated with the key, or an empty
  /// string if not found
  std::string getValue(std::string& key);

  /// @brief Sends a key-value pair to the coprocessor
  /// @param key __std::string&__ The key to send
  /// @param value __std::string&__ The value to send
  void sendValue(std::string& key, const std::string& value);
};
}  // namespace hal
}  // namespace driftless
#endif