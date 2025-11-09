#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"
#include <map>
#include <vector>

/**
 * Arduino code to recieve odometry information from a
 * Sparkfun Odometry sensor (PAA5160E1) and transmits the
 * information to the Vex brain over RS485 connection
 *
 * @author Matthew Backman
 */

enum class EErrorCode {
  RECIEVER_TIMEOUT = 0,
  CRC_MISSMATCH = 1,
  INVALID_KEY = 2
};

constexpr char START_DELIMETER{ '/' };
const std::map<char, uint8_t> key_size{ { 'R', 1 },
                                        { 'X', 4 },
                                        { 'Y', 4 },
                                        { 'H', 4 },
                                        { 'C', 1 } };

bool isValidPackage(const std::vector<uint8_t>& data);
uint16_t computeCRC(const std::vector<uint8_t>& data, uint8_t size);
void handleRequestCommand(const uint8_t* data);
void handleCalibrateCommand(const uint8_t* data);
void handleSetXCommand(const uint8_t* data);
void handleSetYCommand(const uint8_t* data);
void handleSetHeadingCommand(const uint8_t* data);
void sendInvalidPackageError(const uint8_t error_code);

std::map<char, void (*)(const uint8_t*)> command_handlers{ { 'R', handleRequestCommand },
                                                           { 'C', handleCalibrateCommand },
                                                           { 'X', handleSetXCommand },
                                                           { 'Y', handleSetYCommand },
                                                           { 'H', handleSetHeadingCommand } };
// Create an Optical Tracking Odometry Sensor object
QwiicOTOS odom_sensor;

std::vector<char> packets_requested{};

void setup() {
  // put your setup code here, to run once:

  Serial2.begin(74880);
  Serial.begin(74880);
  Wire.begin();

  odom_sensor.setAngularUnit(kSfeOtosAngularUnitDegrees);
  odom_sensor.setLinearUnit(kSfeOtosLinearUnitInches);

  odom_sensor.resetTracking();
}

void loop() {
  // put your main code here, to run repeatedly:

  uint64_t current_time{ millis() };

  packets_requested.clear();

  // Check for input from brain
  if (Serial2.available()) {
    std::vector<uint8_t> recieved_data{};
    recieved_data.push_back(Serial2.read());
    uint8_t bytes_expected{ recieved_data[0] };

    uint32_t recieve_start_time{ millis() };
    while (recieved_data.size() < bytes_expected && millis() < recieve_start_time + 100) {
      if (Serial2.available()) {
        recieved_data.push_back(Serial2.read());
      } else {
        delay(1);
      }
    }

    Serial.write(recieved_data.data(), recieved_data.size());
    Serial.print("\n");

    if (millis() >= recieve_start_time + 100) {
      sendInvalidPackageError(EErrorCode::RECIEVER_TIMEOUT);
      return;
    }

    // validate data
    if (!isValidPackage(recieved_data)) {
      sendInvalidPackageError(EErrorCode::CRC_MISSMATCH);
      return;
    }

    Serial.println("Valid packet recieved");

    uint8_t packets_recieved{ recieved_data[1] };

    uint8_t packet_offset{ 2 };
    for (int i = 0; i < packets_recieved; ++i) {
      char key{ recieved_data[packet_offset++] };
      if (key_size.find(key) == key_size.end()) {
        sendInvalidPackageError(EErrorCode::INVALID_KEY);
        return;
      }
      uint8_t size{ key_size.at(key) };

      const uint8_t* packet_value{ &recieved_data[packet_offset] };

      // handle data
      auto handler = command_handlers.find(key);
      if (handler != command_handlers.end()) {
        handler->second(packet_value);
      }

      packet_offset += size;
    }
  }

  // generate package of data if requested
  if (packets_requested.size()) {
    std::vector<uint8_t> output_package{};
    output_package.push_back(0);
    output_package.push_back(packets_requested.size());
    for (char& requested_key : packets_requested) {
      output_package.push_back(static_cast<uint8_t>(requested_key));

      uint8_t value_size{ key_size.at(requested_key) };
      uint8_t value[value_size];

      switch (requested_key) {
        case 'X':
          {
            sfe_otos_pose2d_t position;
            odom_sensor.getPosition(position);

            memcpy(&value, &position.x, value_size);
            break;
          }
        case 'Y':
          {
            sfe_otos_pose2d_t position;
            odom_sensor.getPosition(position);

            memcpy(&value, &position.y, value_size);
            break;
          }
        case 'H':
          {
            sfe_otos_pose2d_t position;
            odom_sensor.getPosition(position);

            memcpy(&value, &position.h, value_size);
            break;
          }
      }
      output_package.insert(output_package.end(), value, value + key_size.at(requested_key));
    }
    // calculate crc for validation
    uint16_t crc{ computeCRC(output_package, output_package.size()) };
    uint8_t crc_bytes[2];
    memcpy(&crc_bytes, &crc, 2);
    output_package.insert(output_package.end(), crc_bytes, crc_bytes + 2);
    output_package[0] = output_package.size();

    Serial2.write(output_package.data(), output_package.size());
  }

  uint64_t delay_time{ 10 - (millis() - current_time) };
  if (delay_time > 0) {
    delay(delay_time);
  }
}

/** Builds a packet to send over the serial link
 * @tparam T    The datatype of the value sent
 * @param key   The key designating the purpose of the data
 * @param value The value held by the packet 
 */
template<typename T>
String buildPacket(char key, T value) {
  char size{ sizeof(T) };
  char bytes[size]{};
  memcpy(&bytes, &value, size);

  String delimeter_str{ START_DELIMETER };
  String key_str{ key };
  String size_str{ size };
  String value_str{ bytes };

  String output{ delimeter_str + key_str + size_str + value_str };
  return output;
}

/** Computes the CRC of a package
* @param data the base package
* @param length the size of the package
* @return CRC value
*/
uint16_t computeCRC(const std::vector<uint8_t>& data, uint8_t size) {
  uint16_t crc{ 0xFFFF };
  for (size_t i = 0; i < size; ++i) {
    crc ^= static_cast<uint8_t>(data[i]);
    for (uint8_t i = 0; i < 8; ++i) {
      if (crc & 1)
        crc = (crc >> 1) ^ 0xA001;
      else
        crc >>= 1;
    }
  }
  return crc;
}

/** Validates a package by checking the CRC value
* @param data The package recieved
* @return True if the packet was transmitted correctly
*/
bool isValidPackage(const std::vector<uint8_t>& data) {
  uint16_t actual_crc{};

  memcpy(&actual_crc, &data[data.size() - 2], 2);
  uint16_t calculated_crc{ computeCRC(data, data.size() - 2) };

  return actual_crc == calculated_crc;
}

// Define command handlers
void handleRequestCommand(const uint8_t* data) {
  char requested_key{};
  memcpy(&requested_key, data, 1);
  packets_requested.push_back(requested_key);
}

void handleCalibrateCommand(const uint8_t* data) {
  odom_sensor.calibrateImu();
  delay(1000);
}

void handleSetXCommand(const uint8_t* data) {
  float x_offset{};
  memcpy(&x_offset, data, 4);

  sfe_otos_pose2d_t offset{};
  odom_sensor.getOffset(offset);
  offset.x = x_offset;
  odom_sensor.setOffset(offset);
}

void handleSetYCommand(const uint8_t* data) {
  float y_offset{};
  memcpy(&y_offset, data, 4);

  sfe_otos_pose2d_t offset{};
  odom_sensor.getOffset(offset);
  offset.y = y_offset;
  odom_sensor.setOffset(offset);
}

void handleSetHeadingCommand(const uint8_t* data) {
  float h_offset{};
  memcpy(&h_offset, data, 4);

  sfe_otos_pose2d_t offset{};
  odom_sensor.getOffset(offset);
  offset.h = h_offset;
  odom_sensor.setOffset(offset);
}

void sendInvalidPackageError(const EErrorCode error_code) {
  std::vector<uint8_t> package;
  package.reserve(6);
  package.push_back(6);
  package.push_back(1);
  package.push_back(static_cast<uint8_t>('E'));
  package.push_back(static_cast<uint8_t>(error_code));

  uint16_t crc{ computeCRC(package, 4) };
  uint8_t crc_bytes[2];
  memcpy(crc_bytes, &crc, 2);
  package.insert(package.end(), crc_bytes, crc_bytes + 2);

  Serial2.write(package.data(), package.size());
}