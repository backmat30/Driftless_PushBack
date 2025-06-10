#ifndef __COPROCESSOR_HPP__
#define __COPROCESSOR_HPP__

#include <map>
#include <memory>
#include <string>
#include <cstring>

#include "driftless/io/ISerialDevice.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"
#include "driftless/serial_protocol/SerialProtocol.hpp"

namespace driftless {
namespace hal {
class Coprocessor {
 private:
  static constexpr uint8_t TASK_DELAY{10};

  /// @brief Performs task updates in a constant loop
  /// @param params __void*__ Pointer to the coprocessor to update
  static void taskLoop(void* params);

  std::unique_ptr<io::ISerialDevice> m_serial_device;

  std::unique_ptr<rtos::ITask> m_task;

  std::unique_ptr<rtos::IClock> m_clock;

  std::unique_ptr<rtos::IDelayer> m_delayer;

  std::unique_ptr<rtos::IMutex> m_mutex;

  std::map<serial_protocol::ESerialKey, std::string> m_latest_data;

  serial_protocol::SerialProtocol m_serial_protocol;

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

  /// @brief Runs the coprocessor task loop
  void run();

  /// @brief Gets the latest value of a given key, if available
  /// @tparam T The type to convert the bytes to
  /// @param key __ESerialKey__ The key value to search for
  /// @return __T__ The value associated with the key
  template <typename T>
  T getValue(serial_protocol::ESerialKey key);

  /// @brief Sends a key-value pair to the coprocessor
  /// @param key __std::string&__ The key to send
  /// @param value __std::string&__ The value to send
  template <typename T>
  void sendValue(serial_protocol::ESerialKey key, const T& value);

  /// @brief Sets the serial device used by the coprocessor
  /// @param serial_device __std::unique_ptr<io::ISerialDevice>&__ The serial
  /// device to use for communication
  void setSerialDevice(std::unique_ptr<io::ISerialDevice>& serial_device);

  /// @brief Sets the protocol used for serial communication
  /// @param serial_protocol
  /// __std::unique_ptr<serial_protocol::ISerialProtocol>&__ The serial protocol
  /// to use for encoding and decoding messages
  void setSerialProtocol(
      serial_protocol::SerialProtocol& serial_protocol);

  /// @brief Sets the task used for running the update loop
  /// @param task __std::unique_ptr<rtos::ITask>&__ The task to use for running
  /// the update loop
  void setTask(std::unique_ptr<rtos::ITask>& task);

  /// @brief Sets the clock used for timing operations
  /// @param clock __std::unique_ptr<rtos::IClock>&__ The clock to use for
  /// timing operations
  void setClock(std::unique_ptr<rtos::IClock>& clock);

  /// @brief Sets the delayer used for delaying operations
  /// @param delayer __std::unique_ptr<rtos::IDelayer>&__ The delayer to use for
  /// delaying operations
  void setDelayer(std::unique_ptr<rtos::IDelayer>& delayer);

  /// @brief Sets the mutex used for synchronizing access to shared resources
  /// @param mutex __std::unique_ptr<rtos::IMutex>&__ The mutex to use for
  /// synchronizing access to shared resources
  void setMutex(std::unique_ptr<rtos::IMutex>& mutex);
};
}  // namespace hal
}  // namespace driftless
#endif