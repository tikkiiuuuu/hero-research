#ifndef IO__GIMBAL_HPP
#define IO__GIMBAL_HPP

#include "serial/serial.h"
#include "tools/crc.hpp"
#include "tools/thread_safe_queue.hpp"
#include <Eigen/Geometry>
#include <algorithm>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tools/crc.hpp"
#include "tools/logger.hpp"
#include "tools/math_tools.hpp"
#include "tools/yaml.hpp"

#include <bitset>

namespace io {
template<typename... Bases>
struct __attribute__((packed)) Entity: Bases... {};

struct __attribute__((packed)) Header {
    uint8_t head[2] = { 'S', 'P' };
};
struct __attribute__((packed)) Tail {
    uint16_t crc16;
};
struct __attribute__((packed)) BaseRx {
    uint8_t mode; // 0: 空闲, 1: 自瞄, 2: 小符, 3: 大符
    float q[4]; // wxyz顺序
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    uint16_t bullet_count; // 子弹累计发送次数
};
struct __attribute__((packed)) SentryRxExtra {
    float big_yaw;
    uint8_t game_progress;
    uint8_t color; //*红是1,蓝是2
    uint16_t stage_remain_time;
    uint16_t current_hp;
    uint8_t ul_state;
};
struct __attribute__((packed)) BaseTx {
    uint8_t mode; // 0: 不控制, 1: 控制云台但不开火，2: 控制云台且开火
    uint8_t fire;
    float yaw;
    float yaw_vel;
    float yaw_acc;
    float pitch;
    float pitch_vel;
    float pitch_acc;
};

struct __attribute__((packed)) SentryFireTxExtra {
    uint8_t w1; //* 下位机跟踪数等合一块 tracking:1 id:3 armors_num:3 reserved:1
    float armor_distance;
    float true_yaw;
    float armor_angle;
    float center_yaw;
};

struct __attribute__((packed)) SentryNavTxExtra {
    float vx;
    float vy;
    float angle;
};

struct __attribute__((packed)) TimestampExtra {
    float timestamp;
};

//! 目前相同类型一定要绑定使用
using StandardRx = Entity<Header, BaseRx, Tail>;
using SentryRx = Entity<Header, BaseRx, SentryRxExtra, Tail>;
using StandardTx = Entity<Header, BaseTx, Tail>;
using SentryTx = Entity<Header, BaseTx, SentryFireTxExtra, SentryNavTxExtra, Tail>;
// static_assert(sizeof(StandardRx) <= 64);
// static_assert(sizeof(SentryRx) <= 64);
// static_assert(sizeof(StandardTx) <= 64);
// static_assert(sizeof(SentryTx) <= 64);

enum class GimbalMode {
    IDLE, // 空闲
    AUTO_AIM, // 自瞄
    SMALL_BUFF, // 小符
    BIG_BUFF // 大符
};

struct GimbalState {
    float yaw;
    float yaw_vel;
    float pitch;
    float pitch_vel;
    float bullet_speed;
    float big_yaw;
    uint8_t game_progress;
    uint8_t color;
    uint16_t stage_remain_time;
    uint16_t current_hp;
    uint16_t bullet_count;
    uint8_t center_point_status;
    uint8_t rfid_center_point;
    uint8_t rfid_non_supply_overlap;
};

using Ros2Interface = tools::ThreadSafeRingBuffer<std::tuple<float, float, float, int>, 10>;

template<class Rx, class Tx>
class Gimbal {
public:
    explicit Gimbal(
        const std::string& config_path,
        Ros2Interface* interface = nullptr,
        auto_aim::Planner* planner = nullptr
    ) {
        auto yaml = tools::load(config_path);
        com_port_ = tools::read<std::string>(yaml, "com_port");
        if constexpr (std::is_same_v<Rx, SentryRx>) {
            baudrate_ = 921600;
        } else {
            baudrate_ = 115200;
        }

        configure_serial();
        try_open_serial();
        interface_buffer_mode_ = interface;
        thread_ = std::thread(&Gimbal::read_thread, this);
        thread_send_ = std::thread(&Gimbal::thread_send_thread, this);
    }
    ~Gimbal() {
        quit_ = true;
        {
            std::lock_guard<std::mutex> lock(mutex_send_);
            // 唤醒沉睡中的发送线程，让它检查到 quit_ 为 true 并退出循环
            condition_send_.notify_all();
        }
        if (thread_.joinable())
            thread_.join();
        if (thread_send_.joinable())
            thread_send_.join();
        try {
            if (serial_.isOpen())
                serial_.close();
        } catch (const std::exception& e) {
            tools::logger()->warn("[Gimbal] Failed to close serial: {}", e.what());
        }
    }

    GimbalMode mode() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return mode_;
    }
    GimbalState state() const {
        std::lock_guard<std::mutex> lock(mutex_);
        return state_;
    }
    std::string str(GimbalMode mode) const {
        switch (mode) {
            case GimbalMode::IDLE:
                return "IDLE";
            case GimbalMode::AUTO_AIM:
                return "AUTO_AIM";
            case GimbalMode::SMALL_BUFF:
                return "SMALL_BUFF";
            case GimbalMode::BIG_BUFF:
                return "BIG_BUFF";
            default:
                return "INVALID";
        }
    }
    Eigen::Quaterniond q(std::chrono::steady_clock::time_point t) {
        while (true) {
            auto [q_a, t_a] = queue_.pop();
            auto [q_b, t_b] = queue_.front();
            auto t_ab = tools::delta_time(t_a, t_b);
            auto t_ac = tools::delta_time(t_a, t);
            auto k = t_ac / t_ab;
            Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
            if (t < t_a)
                return q_c;
            if (!(t_a < t && t <= t_b))
                continue;

            return q_c;
        }
    }

    std::vector<std::pair<Eigen::Quaterniond, int64_t>> samples_in_range(
        std::chrono::steady_clock::time_point start,
        std::chrono::steady_clock::time_point end,
        std::chrono::steady_clock::time_point origin
    ) const {
        std::lock_guard<std::mutex> lock(raw_samples_mutex_);
        std::vector<std::pair<Eigen::Quaterniond, int64_t>> samples;
        for (const auto& [sample_q, sample_t]: raw_samples_) {
            if (sample_t < start || sample_t > end)
                continue;
            const auto timestamp_ns =
                std::chrono::duration_cast<std::chrono::nanoseconds>(sample_t - origin).count();
            samples.emplace_back(sample_q, timestamp_ns);
        }
        return samples;
    }

    template<class... Args>
    void send(const Args&... args) {
        using DynamicTx = Entity<Header, Args..., Tail>;
        std::lock_guard<std::mutex> tx_lock(mutex_tx_);
        ((static_cast<Args&>(tx_data_) = args), ...);
        //? 这里的意思是通过向基类的转换，得到frame不同部分对应基类部分的引用，再进行变参的赋值
        if constexpr (std::is_base_of_v<BaseTx, DynamicTx>) {
            tx_data_.mode++; //*把0空出来
        }
        aim_mode_ = tx_data_.mode;
        if constexpr (std::is_same_v<Rx, SentryRx>) {
            if (nav_mode_ != 0)
                tx_data_.mode = nav_mode_;
        }
        tx_data_.crc16 = tools::get_crc16(
            reinterpret_cast<uint8_t*>(&tx_data_),
            sizeof(tx_data_) - sizeof(tx_data_.crc16)
        );
        try {
            serial_.write(reinterpret_cast<uint8_t*>(&tx_data_), sizeof(tx_data_));
        } catch (const std::exception& e) {
            tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
        }
    }

    void send_same(bool control) {
        std::lock_guard<std::mutex> tx_lock(mutex_tx_);
        tx_data_.mode = control + 1;
        aim_mode_ = tx_data_.mode;
        if constexpr (std::is_same_v<Rx, SentryRx>) {
            fire_control_extrapolation_.enabled = false;
            if (nav_mode_ != 0)
                tx_data_.mode = nav_mode_;
            tx_data_.w1 = 0;
            tx_data_.armor_distance = 0.0;
            tx_data_.true_yaw = 0.0;
            tx_data_.true_yaw = 1.5;
        }
        tx_data_.fire = 0;
        try {
            serial_.write(reinterpret_cast<uint8_t*>(&tx_data_), sizeof(tx_data_));
        } catch (const std::exception& e) {
            tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
        }
    }

    void update_fire_control_extrapolation(const auto_aim::Plan& plan) {
        if constexpr (std::is_same_v<Rx, SentryRx>) {
            std::lock_guard<std::mutex> tx_lock(mutex_tx_);
            fire_control_extrapolation_.enabled = plan.control;
            fire_control_extrapolation_.base_time = std::chrono::steady_clock::now();
            fire_control_extrapolation_.armor_distance = plan.armor_distance;
            fire_control_extrapolation_.true_yaw = plan.target_yaw;
            fire_control_extrapolation_.armor_angle = plan.armor_angle;
            fire_control_extrapolation_.armor_distance_vel = plan.armor_distance_vel;
            fire_control_extrapolation_.true_yaw_vel = plan.target_yaw_vel;
            fire_control_extrapolation_.armor_angle_vel = plan.armor_angle_vel;
        }
    }

private:
    struct FireControlExtrapolation {
        bool enabled = false;
        std::chrono::steady_clock::time_point base_time {};
        float armor_distance = 0.0f;
        float true_yaw = 0.0f;
        float armor_angle = 0.0f;
        float armor_distance_vel = 0.0f;
        float true_yaw_vel = 0.0f;
        float armor_angle_vel = 0.0f;
    };

    serial::Serial serial_;
    std::string com_port_;
    uint32_t baudrate_ = 115200;

    std::thread thread_;
    std::atomic<bool> quit_ = false;
    mutable std::mutex mutex_;
    std::thread thread_send_;
    std::atomic<bool> thread_send_ready_ = false;

    Rx rx_data_ {};
    Tx tx_data_ {};

    // std::chrono::steady_clock::time_point program_start_time_;
    std::condition_variable condition_send_;
    Ros2Interface* interface_buffer_mode_ = nullptr;
    mutable std::mutex mutex_tx_;

    int nav_mode_ = 0;
    int aim_mode_ = 0;
    mutable std::mutex mutex_send_;
    int time_out_count = 0;
    GimbalMode mode_ = GimbalMode::IDLE;
    GimbalState state_ {};
    FireControlExtrapolation fire_control_extrapolation_;
    tools::ThreadSafeQueue<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
        queue_ { 100 };
    mutable std::mutex raw_samples_mutex_;
    std::deque<std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>> raw_samples_;
    static constexpr size_t max_raw_sample_count_ = 5000;

    void configure_serial() {
        serial_.setPort(com_port_);
        serial_.setBaudrate(baudrate_);
        serial::Timeout time_out = serial::Timeout::simpleTimeout(10);
        serial_.setTimeout(time_out);
    }

    bool try_open_serial() {
        try {
            if (serial_.isOpen())
                return true;

            configure_serial();
            serial_.open();
            serial_.flushInput();
            serial_.flushOutput();
            tools::logger()->info("[Gimbal] Serial opened on {}.", com_port_);
            return true;
        } catch (const std::exception& e) {
            tools::logger()->warn("[Gimbal] Failed to open serial {}: {}", com_port_, e.what());
            return false;
        }
    }

    bool read(uint8_t* buffer, size_t size) {
        try {
            return serial_.read(buffer, size) == size;
        } catch (const std::exception& e) {
            return false;
        }
    }
    bool read_header() {
        uint8_t byte = 0;
        bool saw_s = false;

        while (!quit_) {
            if (!read(&byte, 1))
                return false;

            if (!saw_s) {
                saw_s = byte == 'S';
                continue;
            }

            if (byte == 'P') {
                rx_data_.head[0] = 'S';
                rx_data_.head[1] = 'P';
                return true;
            }

            saw_s = byte == 'S';
        }

        return false;
    }
    void wait_for_first_packet() {
        constexpr auto retry_interval = std::chrono::seconds(5);
        auto last_retry = std::chrono::steady_clock::now();

        while (!quit_) {
            if (!queue_.empty()) {
                tools::logger()->info("[Gimbal] First q received.");
                return;
            }

            const auto now = std::chrono::steady_clock::now();
            if (now - last_retry >= retry_interval) {
                tools::logger()->warn(
                    "[Gimbal] Waiting for first valid packet timed out, reconnecting serial..."
                );
                reconnect();
                last_retry = now;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(20));
        }
    }

    std::tuple<uint8_t, bool, bool> solve_state(uint8_t state) {
        return { (state & 0x3f), (state & 0x40) >> 6, (state & 0x80) >> 7 };
    }

    void read_thread() {
        tools::logger()->info("[Gimbal] read_thread started.");
        int error_count = 0;
        while (!quit_) {
            if (!serial_.isOpen()) {
                reconnect();
                std::this_thread::sleep_for(std::chrono::milliseconds(200));
                continue;
            }
            if (error_count > 5000) {
                error_count = 0;
                tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
                reconnect();
                continue;
            }
            if (!read_header()) {
                error_count++;
                continue;
            }
            auto t = std::chrono::steady_clock::now();
            if (!read(
                    reinterpret_cast<uint8_t*>(&rx_data_) + sizeof(rx_data_.head),
                    sizeof(rx_data_) - sizeof(rx_data_.head)
                ))
            {
                error_count++;
                continue;
            }
            if (!tools::check_crc16(reinterpret_cast<uint8_t*>(&rx_data_), sizeof(rx_data_))) {
                tools::logger()->debug("[Gimbal] CRC16 check failed.");
                error_count++;
                continue;
            }

            error_count = 0;
            Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
            queue_.push({ q, t });
            {
                std::lock_guard<std::mutex> lock(raw_samples_mutex_);
                raw_samples_.emplace_back(q, t);
                while (raw_samples_.size() > max_raw_sample_count_)
                    raw_samples_.pop_front();
            }

            std::lock_guard<std::mutex> lock(mutex_);

            state_.yaw = rx_data_.yaw;
            state_.yaw_vel = rx_data_.yaw_vel;
            state_.pitch = rx_data_.pitch;
            state_.pitch_vel = rx_data_.pitch_vel;
            state_.bullet_speed = rx_data_.bullet_speed;
            state_.bullet_count = rx_data_.bullet_count;
            std::tie(
                state_.center_point_status,
                state_.rfid_center_point,
                state_.rfid_non_supply_overlap
            ) = solve_state(rx_data_.ul_state);

            if constexpr (std::is_same_v<Rx, SentryRx>) {
                state_.big_yaw = rx_data_.big_yaw;
                state_.game_progress = rx_data_.game_progress;
                state_.stage_remain_time = rx_data_.stage_remain_time;
                state_.current_hp = rx_data_.current_hp;
                state_.color = rx_data_.color - 1;
            }

            switch (rx_data_.mode) {
                case 0:
                    mode_ = GimbalMode::IDLE;
                    break;
                case 1:
                    mode_ = GimbalMode::AUTO_AIM;
                    break;
                case 2:
                    mode_ = GimbalMode::SMALL_BUFF;
                    break;
                case 3:
                    mode_ = GimbalMode::BIG_BUFF;
                    break;
                default:
                    mode_ = GimbalMode::IDLE;
                    tools::logger()->warn("[Gimbal] Invalid mode: {}", rx_data_.mode);
                    break;
            }
        }

        tools::logger()->info("[Gimbal] read_thread stopped.");
    }
    void reconnect() {
        while (!quit_) {
            tools::logger()->warn("[Gimbal] Reconnecting serial on {}...", com_port_);
            try {
                if (serial_.isOpen())
                    serial_.close();
            } catch (const std::exception& e) {
                tools::logger()->warn(
                    "[Gimbal] Failed to close serial before reconnect: {}",
                    e.what()
                );
            }

            if (try_open_serial()) {
                queue_.clear();
                tools::logger()->info("[Gimbal] Reconnected serial successfully.");
                return;
            }

            std::this_thread::sleep_for(std::chrono::seconds(1));
        }
    }
    void thread_send_thread() {
        while (!quit_) {
            std::unique_lock<std::mutex> lock(mutex_send_);

            condition_send_.wait_for(lock, std::chrono::milliseconds(5), [this] {
                bool has_work;
                if constexpr (std::is_same_v<Rx, SentryRx>) {
                    has_work = thread_send_ready_.load() && interface_buffer_mode_ != nullptr
                        && !interface_buffer_mode_->empty();
                } else {
                    has_work = thread_send_ready_.load();
                }
                return has_work || quit_;
            });
            if (quit_)
                break;
            //? 这里是短路规则？如果是空指针就直接返回，后续不执行
            if constexpr (std::is_same_v<Rx, SentryRx>) {
                std::lock_guard<std::mutex> tx_lock(mutex_tx_);
                // if (fire_control_extrapolation_.enabled) {
                //     const float dt = std::min(
                //         std::chrono::duration<float>(
                //             std::chrono::steady_clock::now() - fire_control_extrapolation_.base_time
                //         )
                //             .count(),
                //         0.1f
                //     );
                //     tx_data_.armor_distance = std::max(
                //         0.0f,
                //         fire_control_extrapolation_.armor_distance
                //             + fire_control_extrapolation_.armor_distance_vel * dt
                //     );
                //     tx_data_.true_yaw = tools::limit_rad(
                //         fire_control_extrapolation_.true_yaw
                //         + fire_control_extrapolation_.true_yaw_vel * dt
                //     );
                //     tx_data_.armor_angle = tools::limit_rad(
                //         fire_control_extrapolation_.armor_angle
                //         + fire_control_extrapolation_.armor_angle_vel * dt
                //     );
                // }
                if (interface_buffer_mode_ != nullptr && !interface_buffer_mode_->empty()) {
                    auto [x, y, z, mode] = interface_buffer_mode_->back();
                    tx_data_.vx = x;
                    tx_data_.vy = y;
                    tx_data_.angle = z;
                    if (mode != 0) {
                        nav_mode_ = mode;
                        tx_data_.mode = mode;
                        tx_data_.fire = 0;
                    } else {
                        tx_data_.mode = aim_mode_;
                        nav_mode_ = 0;
                    }
                } else {
                    time_out_count++;
                    if (nav_mode_ != 0) {
                        tx_data_.mode = nav_mode_;
                    }
                }
                if (time_out_count > 5000) {
                    time_out_count = 0;
                    tx_data_.vx = 0.0;
                    tx_data_.vy = 0.0;
                    tx_data_.angle = 0.0;
                    nav_mode_ = 0;
                }
                tx_data_.crc16 = tools::get_crc16(
                    reinterpret_cast<uint8_t*>(&tx_data_),
                    sizeof(tx_data_) - sizeof(tx_data_.crc16)
                );
                try {
                    serial_.write(reinterpret_cast<uint8_t*>(&tx_data_), sizeof(tx_data_));
                } catch (const std::exception& e) {
                    tools::logger()->warn(
                        "[Gimbal] Failed to write serial in thread_send_thread: {}",
                        e.what()
                    );
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
};

uint8_t make_w1(bool tracking, uint8_t id, uint8_t armors_num, uint8_t reserved = 0) {
    return ((tracking ? 1 : 0) & 0x01) << 7 | ((id & 0x07) << 4) | ((armors_num & 0x07) << 1)
        | (reserved & 0x01);
}

} // namespace io

#endif // IO__GIMBAL_HPP