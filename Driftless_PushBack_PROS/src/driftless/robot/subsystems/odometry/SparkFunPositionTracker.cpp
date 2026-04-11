#include "driftless/robot/subsystems/odometry/SparkFunPositionTracker.hpp"

#include "pros/screen.hpp"
namespace driftless::robot::subsystems::odometry {
void SparkFunPositionTracker::taskLoop(void* params) {
  SparkFunPositionTracker* instance{
      static_cast<SparkFunPositionTracker*>(params)};

  while (true) {
    instance->taskUpdate();
  }
}

void SparkFunPositionTracker::taskUpdate() {
  uint64_t start_time{m_clock->getTime()};

  updatePosition();
  uint64_t current_time{m_clock->getTime()};
  if (current_time - start_time < TASK_DELAY) {
    m_delayer->delay(TASK_DELAY - (current_time - start_time));
  }
}

void SparkFunPositionTracker::updatePosition() {
  if (m_mutex) {
    m_mutex->take();
  }

  Position raw_position{fetchRawPosition()};
  uint32_t current_time{m_clock->getTime()};

  double x_pos{};
  double y_pos{};
  double heading{};

  x_pos = raw_position.x * std::cos(global_theta_offset) -
          raw_position.y * std::sin(global_theta_offset);
  y_pos = raw_position.x * std::sin(global_theta_offset) +
          raw_position.y * std::cos(global_theta_offset);
  heading = raw_position.theta;

  x_pos += global_x_offset;
  y_pos += global_y_offset;
  heading += global_theta_offset;

  uint32_t time_change{current_time - latest_time};
  double x_change{x_pos - current_position.x};
  double y_change{y_pos - current_position.y};
  double theta_change{heading - current_position.theta};

  if (time_change) {
    current_position.xV = x_change / (time_change * 0.001);
    current_position.yV = y_change / (time_change * 0.001);
    current_position.thetaV = theta_change / (time_change * 0.001);
  }
  current_position.x = x_pos;
  current_position.y = y_pos;
  current_position.theta = heading;

  latest_time = current_time;

  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 1, "X: %7.2f, Y: %7.2f",
                      current_position.x, current_position.y);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER, 3, "Theta: %7.2f ",
                      current_position.theta * 180 / M_PI);

  if (m_mutex) {
    m_mutex->give();
  }
}

Position SparkFunPositionTracker::fetchRawPosition() {
  Position raw_position{};

  if (m_coprocessor) {
    std::array<uint8_t, 8> pos_bytes =
        m_coprocessor->getValue(serial_protocol::ESerialKey::POSITION);

    int16_t raw_x{};
    int16_t raw_y{};
    int16_t raw_theta{};

    std::memcpy(&raw_x, pos_bytes.data(), 2);
    std::memcpy(&raw_y, pos_bytes.data() + 2, 2);
    std::memcpy(&raw_theta, pos_bytes.data() + 4, 2);

    raw_position.x = raw_x / 100.0;
    raw_position.y = raw_y / 100.0;
    raw_position.theta = raw_theta / 100.0 * M_PI / 180;
  }
  return raw_position;
}

void SparkFunPositionTracker::sendLocalOffset() {
  if (m_mutex) {
    m_mutex->take();
  }

  if (m_coprocessor) {
    // m_coprocessor->addPacketToPackage<float>(
    //     serial_protocol::ESerialKey::XPOS,
    //     static_cast<float>(m_local_x_offset));
    // m_coprocessor->addPacketToPackage<float>(
    //     serial_protocol::ESerialKey::YPOS,
    //     static_cast<float>(m_local_y_offset));
    // m_coprocessor->addPacketToPackage<float>(
    //     serial_protocol::ESerialKey::THETA,
    //     static_cast<float>(m_local_theta_offset * 180 / M_PI));

    std::array<uint8_t, 8> position_offset_bytes{};
    int16_t x_offset{static_cast<int16_t>(m_local_x_offset * 100)};
    int16_t y_offset{static_cast<int16_t>(m_local_y_offset * 100)};
    int16_t theta_offset{
        static_cast<int16_t>(m_local_theta_offset * 100 * 180 / M_PI)};
    std::memcpy(position_offset_bytes.data(), &x_offset, 2);
    std::memcpy(position_offset_bytes.data() + 2, &y_offset, 2);
    std::memcpy(position_offset_bytes.data() + 4, &theta_offset, 2);
    m_coprocessor->addPacketToPackage(serial_protocol::ESerialKey::POSITION,
                                      position_offset_bytes);
  }

  if (m_mutex) {
    m_mutex->give();
  }
}

void SparkFunPositionTracker::init() {
  m_coprocessor->init();
  m_delayer->delay(500);
  std::array<uint8_t, 8> calibrate_value{};
  m_coprocessor->addPacketToPackage(serial_protocol::ESerialKey::CALIBRATE_ODOM,
                                    calibrate_value);
  m_delayer->delay(1000);

  sendLocalOffset();

  std::array<uint8_t, 8> pos_request{};
  pos_request[0] = static_cast<uint8_t>(serial_protocol::ESerialKey::POSITION);
  m_coprocessor->addRecurringPacket(serial_protocol::ESerialKey::REQUEST_PACKET,
                                    pos_request);
}

void SparkFunPositionTracker::run() {
  m_task->start(taskLoop, this);
  m_coprocessor->run();
}

void SparkFunPositionTracker::setPosition(Position position) {
  setX(position.x);
  setY(position.y);
  setTheta(position.theta);
}

Position SparkFunPositionTracker::getPosition() { return current_position; }

void SparkFunPositionTracker::setX(double x) {
  global_x_offset += x - current_position.x;
}

void SparkFunPositionTracker::setY(double y) {
  global_y_offset += y - current_position.y;
}

void SparkFunPositionTracker::setTheta(double theta) {
  global_theta_offset += theta - current_position.theta;
}

void SparkFunPositionTracker::setClock(std::unique_ptr<rtos::IClock>& clock) {
  m_clock = std::move(clock);
}

void SparkFunPositionTracker::setDelayer(
    std::unique_ptr<rtos::IDelayer>& delayer) {
  m_delayer = std::move(delayer);
}

void SparkFunPositionTracker::setMutex(std::unique_ptr<rtos::IMutex>& mutex) {
  m_mutex = std::move(mutex);
}

void SparkFunPositionTracker::setTask(std::unique_ptr<rtos::ITask>& task) {
  m_task = std::move(task);
}

void SparkFunPositionTracker::setCoprocessor(
    std::shared_ptr<hal::Coprocessor>& coprocessor) {
  m_coprocessor = coprocessor;
}

void SparkFunPositionTracker::setLocalXOffset(double local_x_offset) {
  m_local_x_offset = local_x_offset;
}

void SparkFunPositionTracker::setLocalYOffset(double local_y_offset) {
  m_local_y_offset = local_y_offset;
}

void SparkFunPositionTracker::setLocalThetaOffset(double local_theta_offset) {
  m_local_theta_offset = local_theta_offset;
}
}  // namespace driftless::robot::subsystems::odometry