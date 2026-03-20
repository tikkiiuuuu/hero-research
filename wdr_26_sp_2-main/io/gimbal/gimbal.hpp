#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include "serial/serial.h"
#include "tools/crc.hpp"
#include "tools/thread_safe_queue.hpp"
#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

namespace io
{

struct __attribute__((packed)) RxData
{
  uint8_t head[2];
  uint8_t mode;  // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
  float q[4];  // wxyz 顺序
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint8_t fire_calm;
  uint8_t enemy_color;
  uint16_t bullet_count;
  uint16_t crc16;
};

struct __attribute__((packed)) TxData
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;  // 0: 不控制, 1: 控制云台但不开火, 2: 控制云台且开火
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  float dist;
  uint16_t crc16;
};

struct __attribute__((packed)) TxDataSentry
{
  uint8_t head[2] = {'S', 'P'};
  uint8_t mode;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  float big_yaw;
  float x;
  float y;
  float z;
  float dir1;
  float dir2;
  float dir3;
  uint16_t crc16;
};

struct __attribute__((packed)) RxDataTimestamp
{
  uint8_t head[2];
  float timestamp;
  uint16_t crc16;
};

struct __attribute__((packed)) TxDataTimestamp
{
  uint8_t head[2] = {'S', 'P'};
  float timestamp;
  uint16_t crc16;
};

enum class GimbalMode
{
  IDLE,
  AUTO_AIM,
  SMALL_BUFF,
  BIG_BUFF
};

struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  bool fire_calm;
  uint8_t enemy_color;
  uint16_t bullet_count;
};

struct VisionToGimbal
{
  uint8_t mode;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;
  float dist = 0.0f;
};

using Ros2Interface = tools::ThreadSafeRingBuffer<std::tuple<float, float, float>, 10>;

class Gimbal
{
public:
  explicit Gimbal(const std::string & config_path);
  Gimbal(const std::string & config_path, int use_timestamp);
  ~Gimbal();

  GimbalMode mode() const;
  GimbalState state() const;
  std::string str(GimbalMode mode) const;
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  void set_interface(Ros2Interface & interface)
  {
    interface_buffer_ = &interface;
  }

  void send_lose(io::GimbalState state);
  void send_lose_ui(io::GimbalState state, double dist);
  void send(io::VisionToGimbal vision_to_gimbal);
  void send(
    bool control,
    bool fire,
    float yaw,
    float yaw_vel,
    float yaw_acc,
    float pitch,
    float pitch_vel,
    float pitch_acc);
  void send_ui(
    bool control,
    bool fire,
    float yaw,
    float yaw_vel,
    float yaw_acc,
    float pitch,
    float pitch_vel,
    float pitch_acc,
    double dist);
  void send_sentry(
    bool control,
    bool fire,
    float yaw,
    float yaw_vel,
    float yaw_acc,
    float pitch,
    float pitch_vel,
    float pitch_acc,
    float big_yaw,
    float x,
    float y,
    float z,
    float dir1,
    float dir2);
  void send_timestamp(std::chrono::steady_clock::time_point t);

  tools::ThreadSafeQueue<std::tuple<float, float>> queue_timestamp_ {100};

private:
  bool read(uint8_t * buffer, size_t size);
  void read_thread();
  void read_thread_timestamp();
  void reconnect();
  void thread_send_thread();

  serial::Serial serial_;

  std::thread thread_;
  std::thread thread_send_;
  std::atomic<bool> quit_ = false;
  std::atomic<bool> thread_send_ready_ = true;

  mutable std::mutex mutex_;
  mutable std::mutex mutex_send_;
  std::condition_variable condition_send_;

  GimbalMode mode_ = GimbalMode::IDLE;
  GimbalState state_ {};
  RxData rx_data_ {};
  TxData tx_data_ {};
  TxDataSentry tx_data_sentry_ {};
  RxDataTimestamp rx_data_timestamp_ {};
  TxDataTimestamp tx_data_timestamp_ {};

  tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_ {100};

  Ros2Interface * interface_buffer_ = nullptr;
  std::chrono::steady_clock::time_point program_start_time_;
};

}  // namespace io

#endif  // IO__GIMBAL_HPP