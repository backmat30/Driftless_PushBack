#ifndef __PACKAGE_MANAGER_H__
#define __PACKAGE_MANAGER_H__

#include <array>
#include <Arduino.h>

#include "CircularBuffer.h"

/**
* Class to manage packets being sent to and recieved from the vex brain
*/
class PackageManager {
  private:
    enum class States{
    WAIT_FOR_DELIMITER,
    READ_SIZE,
    READ_PAYLOAD,
    VALIDATE_PACKAGE,
    PROCESS_COMMANDS,
    BUILD_RESPONSE,
    SEND_RESPONSE,
    ERROR
    };

    CircularBuffer<1024> m_input_buffer{};

    std::array<uint8_t, 1024> m_input_package{};

    std::array<uint8_t, 128> m_packet_sizes{};

    std::array<void (*)(const uint8_t* data), 128> m_packet_handlers{};

    std::array<uint8_t, 1024> m_output_buffer{};

    std::array<uint8_t, 1024> m_output_package{};

    States m_state{States::WAIT_FOR_DELIMITER};

    uint32_t m_state_elapsed_time{};

    uint8_t m_expected_package_size{};

    uint8_t m_bytes_read{};

    size_t m_output_size{};

    size_t m_input_size{};

    size_t m_next_encoded_zero{};

    HardwareSerialIMXRT* m_serial_port{};

    uint16_t calculateCRC(uint8_t* data, size_t size);

    void encodeCOBS();

    bool decodeCOBS();

    bool waitForDelimeter();

    bool readSize();

    bool readPayload();

    bool validatePackage();

    bool processCommands();

    bool buildResponse();

    bool sendResponse();

    bool sendError();

  public:
    PackageManager(HardwareSerialIMXRT* serial_port);

    void addPacketType(char key, uint8_t size, void (*)(uint8_t*) handler);

    void update();
};

#endif