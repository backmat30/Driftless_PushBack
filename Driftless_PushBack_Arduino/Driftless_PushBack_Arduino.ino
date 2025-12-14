#include <map>
#include <vector>

#include "SparkFun_Qwiic_OTOS_Arduino_Library.h"
#include "Wire.h"

// #define DEBUG

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

constexpr char START_DELIMETER{'/'};
const std::map<char, uint8_t> key_size{
    {'R', 1}, {'X', 4}, {'Y', 4}, {'H', 4}, {'C', 1}};

bool isValidPackage(const std::vector<uint8_t>& data);
uint16_t computeCRC(const std::vector<uint8_t>& data, uint8_t size);
void handleRequestCommand(const uint8_t* data);
void handleCalibrateCommand(const uint8_t* data);
void handleSetXCommand(const uint8_t* data);
void handleSetYCommand(const uint8_t* data);
void handleSetHeadingCommand(const uint8_t* data);
void sendInvalidPackageError(const uint8_t error_code);

std::map<char, void (*)(const uint8_t*)> command_handlers{
    {'R', handleRequestCommand},
    {'C', handleCalibrateCommand},
    {'X', handleSetXCommand},
    {'Y', handleSetYCommand},
    {'H', handleSetHeadingCommand}};
// Create an Optical Tracking Odometry Sensor object
QwiicOTOS odom_sensor;

std::vector<char> packets_requested{};

void setup() {
  // put your setup code here, to run once:

  Serial8.begin(115200);

#ifdef DEBUG
  Serial.begin(74880);
#endif

  Wire.begin();
  while (!(odom_sensor.begin())) {
#ifdef DEBUG
    Serial.println("Odom initializing");
#endif
    delay(1000);
  }

  odom_sensor.setAngularUnit(kSfeOtosAngularUnitDegrees);
  odom_sensor.setLinearUnit(kSfeOtosLinearUnitInches);

  odom_sensor.resetTracking();
}

void loop() {
  // Check for input from brain
  packets_requested.clear();

  std::vector<uint8_t> recieved_data;
  uint32_t start_time = millis();
  bool found_delim = false;

  // Hunt for 0xFF (discard garbage)
  while (!found_delim && millis() < start_time + 15) {
    if (Serial8.available()) {
      uint8_t b = Serial8.read();
      if (b == 0xFF) {
        recieved_data.push_back(b);
        found_delim = true;
      }
    }
  }
  if (!found_delim) {
    return;
  }

  // Read size byte
  start_time = millis();
  while (recieved_data.size() < 2 && millis() < start_time + 15) {
    if (Serial8.available()) {
      recieved_data.push_back(Serial8.read());
    }
  }
  if (recieved_data.size() < 2) {
#ifdef DEBUG
    Serial.println("Timeout on size byte");
#endif

    sendInvalidPackageError(EErrorCode::RECIEVER_TIMEOUT);
    return;
  }
  uint8_t expected_size = recieved_data[1];
  if (expected_size < 4) {  // Min: 0xFF + size + count + CRC (empty package)
#ifdef DEBUG
    Serial.println("Invalid size");
#endif

    sendInvalidPackageError(EErrorCode::CRC_MISSMATCH);  // Reuse as invalid
    return;
  }

  // Read remaining bytes
  uint8_t remaining = expected_size - 2;
  start_time = millis();
  while (remaining > 0 && millis() < start_time + 15) {
    if (Serial8.available()) {
      recieved_data.push_back(Serial8.read());
      remaining--;
    }
  }
  if (remaining > 0) {
#ifdef DEBUG
    Serial.println("Timeout on data");
#endif

    sendInvalidPackageError(EErrorCode::RECIEVER_TIMEOUT);
    return;
  }

  // validate
  if (recieved_data.size() != expected_size || recieved_data[0] != 0xFF) {
#ifdef DEBUG
    Serial.println("Size or delim mismatch");
#endif

    sendInvalidPackageError(EErrorCode::CRC_MISSMATCH);
    return;
  }
  if (!isValidPackage(recieved_data)) {
#ifdef DEBUG
    Serial.println("Invalid CRC");
#endif

    sendInvalidPackageError(EErrorCode::CRC_MISSMATCH);
    return;
  }

// Process packets (your existing code from here)
#ifdef DEBUG
  Serial.println("Valid packet recieved");
#endif

  Serial8.flush();

  uint8_t packets_recieved = recieved_data[2];
  uint8_t packet_offset = 3;
  for (int i = 0; i < packets_recieved; ++i) {
    char key = recieved_data[packet_offset++];
    if (key_size.find(key) == key_size.end()) {
      sendInvalidPackageError(EErrorCode::INVALID_KEY);
#ifdef DEBUG
      Serial.println(key);
#endif

      break;
    }
    uint8_t size = key_size.at(key);
    const uint8_t* packet_value = &recieved_data[packet_offset];

    auto handler = command_handlers.find(key);
    if (handler != command_handlers.end()) {
      handler->second(packet_value);
    }

    packet_offset += size;
#ifdef DEBUG
    Serial.println("packet " + String(i) + " of " + String(packets_recieved) +
                   " read");
#endif
  }

  // generate package of data if requested
  if (packets_requested.size() > 0) {
    std::vector<uint8_t> output_package{};
    output_package.push_back(0xff);
    output_package.push_back(0);
    output_package.push_back(packets_requested.size());
    for (char& requested_key : packets_requested) {
      output_package.push_back(static_cast<uint8_t>(requested_key));

      uint8_t value_size{key_size.at(requested_key)};
      uint8_t value[value_size];
      switch (requested_key) {
        case 'X': {
          sfe_otos_pose2d_t position;
          odom_sensor.getPosition(position);
          float xPos = position.x;

          memcpy(&value, &xPos, value_size);
          break;
        }
        case 'Y': {
          sfe_otos_pose2d_t position;
          odom_sensor.getPosition(position);
          float yPos = position.y;

          memcpy(&value, &yPos, value_size);
          break;
        }
        case 'H': {
          sfe_otos_pose2d_t position;
          odom_sensor.getPosition(position);
          float heading = position.h;

          memcpy(&value, &heading, value_size);
          break;
        }
      }
#ifdef DEBUG
      Serial.print("Value of: " + String(requested_key) + ": ");
      for (int i = 0; i < value_size; ++i) {
        Serial.print(value[i], HEX);
      }
      Serial.println();
#endif

      output_package.insert(output_package.end(), value, value + value_size);
    }
    output_package[1] = output_package.size() + 2;

    // calculate crc for validation
    uint16_t crc{computeCRC(output_package, output_package.size())};
    uint8_t crc_bytes[2];
    memcpy(&crc_bytes, &crc, 2);
    output_package.insert(output_package.end(), crc_bytes, crc_bytes + 2);

#ifdef DEBUG
    Serial.print("Sending: ");
    for (auto& byte : output_package) {
      Serial.print(byte, HEX);
      Serial.print(" ");
    }
    Serial.println("");
    Serial.println("");
#endif

    for (auto& byte : output_package) {
      Serial8.write(byte);
    }
  }

  int32_t delay_time{10 - (millis() - start_time)};
  if (delay_time > 0) {
    delay(delay_time);
  }
}

/** Computes the CRC of a package
 * @param data the base package
 * @param length the size of the package
 * @return CRC value
 */
uint16_t computeCRC(const std::vector<uint8_t>& data, uint8_t size) {
  uint16_t crc{0xFFFF};
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
  uint16_t calculated_crc{computeCRC(data, data.size() - 2)};

  return actual_crc == calculated_crc;
}

// Define command handlers
void handleRequestCommand(const uint8_t* data) {
  char requested_key{};
  memcpy(&requested_key, data, 1);
  packets_requested.push_back(requested_key);
}

void handleCalibrateCommand(const uint8_t* data) {
  odom_sensor.resetTracking();
  odom_sensor.calibrateImu(255);
#ifdef DEBUG
  Serial.println("Calibrating");
#endif
}

void handleSetXCommand(const uint8_t* data) {
  float x_offset{};
  memcpy(&x_offset, data, 4);

#ifdef DEBUG
  Serial.println("X offset: " + String(x_offset));
#endif

  sfe_otos_pose2d_t offset{};
  odom_sensor.getOffset(offset);
  offset.x = x_offset;
  odom_sensor.setOffset(offset);
}

void handleSetYCommand(const uint8_t* data) {
  float y_offset{};
  memcpy(&y_offset, data, 4);

#ifdef DEBUG
  Serial.println("Y offset: " + String(y_offset));
#endif

  sfe_otos_pose2d_t offset{};
  odom_sensor.getOffset(offset);
  offset.y = y_offset;
  odom_sensor.setOffset(offset);
}

void handleSetHeadingCommand(const uint8_t* data) {
  float h_offset{};
  memcpy(&h_offset, data, 4);

#ifdef DEBUG
  Serial.println("Heading offset: " + String(h_offset));
#endif

  sfe_otos_pose2d_t offset{};
  odom_sensor.getOffset(offset);
  offset.h = h_offset;
  odom_sensor.setOffset(offset);
}

void sendInvalidPackageError(const EErrorCode error_code) {
  std::vector<uint8_t> package;
  package.reserve(7);
  package.push_back(0xff);
  package.push_back(7);
  package.push_back(1);
  package.push_back(static_cast<uint8_t>('E'));
  package.push_back(static_cast<uint8_t>(error_code));

  uint16_t crc{computeCRC(package, 4)};
  uint8_t crc_bytes[2];
  memcpy(crc_bytes, &crc, 2);
  package.insert(package.end(), crc_bytes, crc_bytes + 2);

  Serial8.write(package.data(), package.size());
#ifdef DEBUG
  Serial.println("");
  Serial.println(
      "=============================================\n========================="
      "====================");
  Serial.println("Error: " + String(static_cast<int>(error_code)));
  Serial.println(
      "=============================================\n========================="
      "====================");
  Serial.println("");
#endif DEBUG
}