#ifndef __COPROCESSOR_HPP__
#define __COPROCESSOR_HPP__

#include <array>
#include <cstring>
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

  enum class EStates {
    WAIT_FOR_DELIMITER,
    READ_PAYLOAD,
    VALIDATE_PACKAGE,
    PROCESS_COMMANDS,
    ERROR
  };

  static constexpr uint16_t TASK_DELAY{20};

  /// @brief Performs task updates in a constant loop
  /// @param params __void*__ Pointer to the coprocessor to update
  static void taskLoop(void* params);

  /// @brief Handles the X position command
  /// @param data __const uint8_t*__ The data received for the command
  /// @param coprocessor __Coprocessor*__ Pointer to the coprocessor object
  static void handleXposCommand(const uint8_t* data, Coprocessor* coprocessor);

  /// @brief Handles the Y position command
  /// @param data __const uint8_t*__ The data received for the command
  /// @param coprocessor __Coprocessor*__ Pointer to the coprocessor object
  static void handleYposCommand(const uint8_t* data, Coprocessor* coprocessor);

  /// @brief Handles the theta command
  /// @param data __const uint8_t*__ The data received for the command
  /// @param coprocessor __Coprocessor*__ Pointer to the coprocessor object
  static void handleThetaCommand(const uint8_t* data, Coprocessor* coprocessor);

  static void handlePosCommand(const uint8_t* data, Coprocessor* coprocessor);

  /// @brief Handles the acknowledgment command
  /// @param data __const uint8_t*__ The data received for the command
  /// @param coprocessor __Coprocessor*__ Pointer to the coprocessor object
  static void handleAckCommand(const uint8_t* data, Coprocessor* coprocessor);

  std::unique_ptr<io::ISerialDevice> m_serial_device;

  std::unique_ptr<rtos::ITask> m_task;

  std::unique_ptr<rtos::IClock> m_clock;

  std::unique_ptr<rtos::IDelayer> m_delayer;

  std::unique_ptr<rtos::IMutex> m_mutex;

  std::array<std::array<uint8_t, 8>, 128> m_latest_data{};

  std::array<bool, 128> m_has_data{};

  std::array<uint8_t, 255> m_input_package{};

  std::array<uint8_t, 255> m_decoded_input_package{};

  std::array<uint8_t, 128> m_packet_sizes{};

  std::array<void (*)(const uint8_t* read_data, Coprocessor*), 128>
      m_packet_handlers{};

  std::array<uint8_t, 255> m_output_buffer{};

  serial_protocol::Package m_outgoing_package{};

  std::vector<serial_protocol::Packet> m_recurring_packets{};

  std::vector<serial_protocol::Packet> m_packet_queue{};

  EStates m_state{EStates::WAIT_FOR_DELIMITER};

  uint32_t m_state_elapsed_time{};

  uint8_t m_expected_package_size{};

  uint8_t m_bytes_read{};

  uint8_t m_output_packets{};

  size_t m_output_size{};

  bool m_connection_established{};

  uint16_t calculateCRC(uint8_t* data, size_t size);

  void encodeCOBS();

  bool decodeCOBS();

  bool waitForDelimiter();

  bool readPayload();

  bool validatePackage();

  bool processCommands();

  bool buildResponse();

  bool sendResponse();

  bool handleError();

  void taskUpdate();

 public:
  Coprocessor();

  /// @brief Initializes the Coprocessor object
  void init();

  /// @brief Runs the coprocessor task loop
  void run();

  /// @brief Gets the latest value of a given key, if available
  /// @param key __ESerialKey__ The key value to search for
  /// @return __std::array<uint8_t, 8>__ The value associated with the key
  std::array<uint8_t, 8> getValue(serial_protocol::ESerialKey key) {
    if (m_mutex) {
      m_mutex->take();
    }

    std::array<uint8_t, 8> value{};

    if (m_connection_established && m_has_data[static_cast<uint8_t>(key)]) {
      value = m_latest_data[static_cast<uint8_t>(key)];
    }

    if (m_mutex) {
      m_mutex->give();
    }

    return value;
  }

  /// @brief Adds a packet to the next outgoing package
  /// @param key __serial_protocol::ESerialKey__ The key associated with the
  /// value
  /// @param value __std::array<uint8_t, 8>__ The value to send
  void addPacketToPackage(serial_protocol::ESerialKey key,
                          const std::array<uint8_t, 8>& value) {
    if (m_mutex) {
      m_mutex->take();
    }

    std::vector<uint8_t> byte_vector(m_packet_sizes[static_cast<uint8_t>(key)]);
    std::memcpy(byte_vector.data(), value.data(), byte_vector.size());
    m_outgoing_package.addPacket(key, byte_vector);

    if (m_mutex) {
      m_mutex->give();
    }
  }

  /// @brief Adds a packet to the list of packets to be sent for every package
  /// @param key __serial_protocol::ESerialKey__ The key associated with the
  /// value
  /// @param value __std::array<uint8_t, 8>__ The value to send
  void addRecurringPacket(serial_protocol::ESerialKey key,
                          const std::array<uint8_t, 8>& value) {
    if (m_mutex) {
      m_mutex->take();
    }

    std::vector<uint8_t> byte_vector(m_packet_sizes[static_cast<uint8_t>(key)]);
    std::memcpy(byte_vector.data(), value.data(), byte_vector.size());
    serial_protocol::Packet packet{key, byte_vector};

    m_recurring_packets.push_back(packet);

    if (m_mutex) {
      m_mutex->give();
    }
  }

  void removeRecurringPacket(serial_protocol::ESerialKey key,
                             const std::array<uint8_t, 8>& value) {
    if (m_mutex) {
      m_mutex->take();
    }

    for (int i = 0; i < m_recurring_packets.size(); ++i) {
      if (m_recurring_packets[i].key == key) {
        m_recurring_packets.erase(m_recurring_packets.begin() + i);
        break;
      }
    }

    if (m_mutex) {
      m_mutex->give();
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