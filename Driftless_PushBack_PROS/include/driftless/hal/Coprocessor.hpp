#ifndef __COPROCESSOR_HPP__
#define __COPROCESSOR_HPP__

#include <cstring>
#include <map>
#include <memory>
#include <string>

#include "driftless/io/ISerialDevice.hpp"
#include "driftless/rtos/IClock.hpp"
#include "driftless/rtos/IDelayer.hpp"
#include "driftless/rtos/IMutex.hpp"
#include "driftless/rtos/ITask.hpp"
#include "driftless/serial_protocol/Package.hpp"
#include "pros/screen.hpp"

namespace driftless {
namespace hal {
class Coprocessor {
 private:
  enum class EErrorCode : uint8_t {
    FETCH_TIMEOUT = 0,
    INVALID_CRC = 1,
    INVALID_KEY = 2
  };

  static constexpr uint16_t TASK_DELAY{20};

  /// @brief Performs task updates in a constant loop
  /// @param params __void*__ Pointer to the coprocessor to update
  static void taskLoop(void* params);

  /// @brief Handles the X position command
  /// @param data __const uint8_t*__ The data received for the command
  void handleXposCommand(const uint8_t* data);

  /// @brief Handles the Y position command
  /// @param data __const uint8_t*__ The data received for the command
  void handleYposCommand(const uint8_t* data);

  /// @brief Handles the theta command
  /// @param data __const uint8_t*__ The data received for the command
  void handleThetaCommand(const uint8_t* data);

  const std::map<serial_protocol::ESerialKey,
                 void (Coprocessor::*)(const uint8_t*)>
      m_command_handlers{
          {serial_protocol::ESerialKey::XPOS, &Coprocessor::handleXposCommand},
          {serial_protocol::ESerialKey::YPOS, &Coprocessor::handleYposCommand},
          {serial_protocol::ESerialKey::THETA,
           &Coprocessor::handleThetaCommand},
      };

  const std::map<serial_protocol::ESerialKey, uint8_t> m_command_sizes{
      {serial_protocol::ESerialKey::XPOS, sizeof(float)},
      {serial_protocol::ESerialKey::YPOS, sizeof(float)},
      {serial_protocol::ESerialKey::THETA, sizeof(float)},
  };

  std::unique_ptr<io::ISerialDevice> m_serial_device;

  std::unique_ptr<rtos::ITask> m_task;

  std::unique_ptr<rtos::IClock> m_clock;

  std::unique_ptr<rtos::IDelayer> m_delayer;

  std::unique_ptr<rtos::IMutex> m_mutex;

  std::map<serial_protocol::ESerialKey, std::vector<uint8_t>> m_latest_data;

  std::vector<uint8_t> m_serial_buffer;

  serial_protocol::Package m_outgoing_package{};

  std::vector<serial_protocol::Packet> m_recurring_packets{};

  /// @brief Updates the coprocessor object
  void taskUpdate();

  /// @brief Fetches the latest signal from the coprocessor and stores it as
  /// key-value pairs
  void fetchLatestSignal();

  /// @brief finds and stores packets recieved from the latest signal
  void processLatestSignal();

  /// @brief Determines if the buffer contains a valid signal
  /// @return __bool__ True if the signal is valid, false otherwise
  bool isValidSignal() const;

  /// @brief Calculates the CRC for a set of data
  /// @param data __vector<uint8_t>&__ The data to calculate the CRC for
  /// @param length __uint8_t__ The length of the data
  /// @return __uint16_t__ The calculated CRC value
  uint16_t calculateCRC(const std::vector<uint8_t>& data, uint8_t length) const;

  /// @brief Sends the outgoing package to the coprocessor
  void sendOutgoingPackage();

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
  T getValue(serial_protocol::ESerialKey key) {
    T value{};

    if (m_latest_data.contains(key)) {
      try {
        std::memcpy(&value, m_latest_data[key].data(),
                    sizeof(T));  // Copy the bytes from the string to the value
      } catch (const std::exception& e) {
        // Handle conversion error, e.g., log it or throw an exception
        // For now, we will just return the default value
        pros::screen::print(pros::E_TEXT_MEDIUM_CENTER, 10,
                            "Invalid conversion for %d at %7.2f",
                            static_cast<char>(key), m_clock->getTime() / 1000.0);
        value = T{};
      }
    }

    return value;
  }

  /// @brief Adds a packet to the next outgoing package
  /// @tparam T The type of the value to send
  /// @param key __serial_protocol::ESerialKey__ The key associated with the
  /// value
  /// @param value __T__ The value to send
  template <typename T>
  void addPacketToPackage(serial_protocol::ESerialKey key,
                                       const T& value) {
    std::vector<uint8_t> byte_vector(sizeof(T));
    std::memcpy(byte_vector.data(), &value, sizeof(T));
    m_outgoing_package.addPacket(key, byte_vector);
  }

  /// @brief Adds a packet to the list of packets to be sent for every package
  /// @tparam T The type of the value to send
  /// @param key __serial_protocol::ESerialKey__ The key associated with the
  /// value
  /// @param value __T__ The value to send
  template <typename T>
  void addRecurringPacket(serial_protocol::ESerialKey key,
                                       const T& value) {
    std::vector<uint8_t> byte_vector(sizeof(T));
    std::memcpy(byte_vector.data(), &value, sizeof(T));
    serial_protocol::Packet packet{key, byte_vector};

    m_recurring_packets.push_back(packet);
  }

  template <typename T>
  void removeRecurringPacket(serial_protocol::ESerialKey key,
                                          const T& value) {
    std::vector<uint8_t> byte_vector(sizeof(T));
    std::memcpy(byte_vector.data(), &value, sizeof(T));
    serial_protocol::Packet packet{key, byte_vector};

    for (int i = 0; i < m_recurring_packets.size(); ++i) {
      if (m_recurring_packets[i] == packet) {
        m_recurring_packets.erase(m_recurring_packets.begin() + i);
        break;
      }
    }
  }

  /// @brief Sets the serial device used by the coprocessor
  /// @param serial_device __std::unique_ptr<io::ISerialDevice>&__ The serial
  /// device to use for communication
  void setSerialDevice(std::unique_ptr<io::ISerialDevice>& serial_device);

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