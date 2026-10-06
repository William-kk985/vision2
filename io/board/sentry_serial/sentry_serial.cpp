/**
 * @file  io/board/sentry_serial/sentry_serial.cpp
 * @brief 哨兵串口驱动实现。协议见 `sentry_serial.hpp` 文件头注释。
 */
#include "sentry_serial.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <system_error>
#include <vector>

#include "utils/log/logger.hpp"
#include "utils/math/math_tools.hpp"
#include "utils/yaml/yaml.hpp"

namespace io
{
namespace
{

/// @brief 列出本机可用串口（打不开串口时给用户看，照 `io/board/gimbal/gimbal.cpp` 的做法）
std::vector<std::string> list_serial_ports()
{
  std::vector<std::string> avail;
  std::error_code ec;
  for (const auto & e : std::filesystem::directory_iterator("/dev", ec)) {
    const auto n = e.path().filename().string();
    if (n.rfind("ttyUSB", 0) == 0 || n.rfind("ttyACM", 0) == 0 || n.rfind("ttyS", 0) == 0)
      avail.push_back(e.path().string());
  }
  std::sort(avail.begin(), avail.end());
  return avail;
}

/// @brief 读 yaml 的可选 float 键（缺省返回 fallback，**不 exit**）
float read_optional_float(const YAML::Node & yaml, const char * key, float fallback)
{
  if (yaml[key]) return yaml[key].as<float>();
  return fallback;
}

/// @brief 读 yaml 的可选 string 键（缺省返回 fallback）
std::string read_optional_string(
  const YAML::Node & yaml, const char * key, const std::string & fallback)
{
  if (yaml[key]) return yaml[key].as<std::string>();
  return fallback;
}

}  // namespace

SentrySerial::SentrySerial(const std::string & config_path)
{
  auto yaml = tools::load(config_path);

  // ⭐ 端口：`serial_port` 优先，回退 `com_port`（兼容参考实现与现有 yaml）
  port_name_ = read_optional_string(yaml, "serial_port", "");
  if (port_name_.empty()) port_name_ = read_optional_string(yaml, "com_port", "");
  if (port_name_.empty()) {
    tools::logger()->error(
      "[SentrySerial] 配置里没有 `serial_port` 或 `com_port`（来自 {}）⇒ 串口不会打开", config_path);
    return;
  }

  // ⭐ 偏置从 yaml 读（度），**不硬编码** —— 参考实现里写死 −1.8°/−6.6°
  // ⚠️ 本文件不依赖 OpenCV ⇒ 不用 CV_PI，用 M_PI（<cmath>）
  constexpr float kDeg2Rad = static_cast<float>(M_PI) / 180.0f;
  yaw_offset_ = read_optional_float(yaml, "yaw_offset", 0.0f) * kDeg2Rad;
  pitch_offset_ = read_optional_float(yaml, "pitch_offset", 0.0f) * kDeg2Rad;

  tools::logger()->info(
    "[SentrySerial] 打开 {} @115200 | 上行 {} 字节 / 下行 {} 字节 | 偏置 yaw={:.2f}° pitch={:.2f}°",
    port_name_, sizeof(SentryToVisionPacket), sizeof(VisionToSentryPacket),
    yaw_offset_ / kDeg2Rad, pitch_offset_ / kDeg2Rad);

  try {
    serial_.setPort(port_name_);
    serial_.setBaudrate(115200);
    serial::Timeout timeout = serial::Timeout::simpleTimeout(100);
    serial_.setTimeout(timeout);
    serial_.open();
    connected_ = true;
    tools::logger()->info("[SentrySerial] 串口已打开");
  } catch (const std::exception & e) {
    // ⭐ 把错误变成可操作的（沿用本项目 W53 的做法）：说清试了哪个设备、机器上有哪些、怎么办
    connected_ = false;
    tools::logger()->error("[SentrySerial] 打不开串口: {}", e.what());
    tools::logger()->error("[SentrySerial]  尝试的设备: '{}'（来自 {}）", port_name_, config_path);
    const auto avail = list_serial_ports();
    if (avail.empty()) {
      tools::logger()->error("[SentrySerial]  /dev 下没有 ttyUSB*/ttyACM*/ttyS* ⇒ 下位机没接或没驱动");
    } else {
      tools::logger()->error("[SentrySerial]  本机可用串口: {}", fmt::join(avail, " "));
    }
    return;   // ⚠️ 不 exit：录像/无硬件时仍可跑（与 `make_board` 的哲学一致）
  }

  thread_ = std::thread(&SentrySerial::read_thread, this);
}

SentrySerial::~SentrySerial()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  if (serial_.isOpen()) {
    try {
      serial_.close();
    } catch (...) {
    }
  }
}

SentryMode SentrySerial::mode() const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return mode_;
}

SentryState SentrySerial::state() const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return state_;
}

const char * SentrySerial::mode_str(SentryMode m)
{
  switch (m) {
    case SentryMode::IDLE: return "IDLE";
    case SentryMode::AUTO_AIM: return "AUTO_AIM";
    case SentryMode::SMALL_BUFF: return "SMALL_BUFF";
    case SentryMode::BIG_BUFF: return "BIG_BUFF";
  }
  return "UNKNOWN";
}

Eigen::Quaterniond SentrySerial::latest_q() const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return latest_q_;
}

SentryToVisionPacket SentrySerial::last_rx_packet() const
{
  std::lock_guard<std::mutex> lk(mutex_);
  return rx_data_;
}

Eigen::Quaterniond SentrySerial::q(std::chrono::steady_clock::time_point t)
{
  if (queue_.empty()) return latest_q();

  while (true) {
    auto [q_a, t_a] = queue_.pop();
    if (queue_.empty()) return q_a;

    auto [q_b, t_b] = queue_.front();
    const auto t_ab = tools::delta_time(t_a, t_b);
    const auto t_ac = tools::delta_time(t_a, t);
    if (t_ab <= 0) return q_a;

    const auto k = std::clamp(t_ac / t_ab, 0.0, 1.0);
    const Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();

    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;
    return q_c;
  }
}

void SentrySerial::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc, float nav_x, float nav_y, float nav_w, uint8_t status_position)
{
  if (!connected_.load() || !serial_.isOpen()) return;

  tx_data_.head[0] = FRAME_HEAD[0];
  tx_data_.head[1] = FRAME_HEAD[1];
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  // ⭐ 偏置从 yaml 来（参考实现里是硬编码常量）
  tx_data_.yaw = yaw + yaw_offset_;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch + pitch_offset_;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.nav_x = nav_x;
  tx_data_.nav_y = nav_y;
  tx_data_.nav_w = nav_w;
  tx_data_.status_position = status_position;
  tx_data_.tail[0] = FRAME_TAIL[0];
  tx_data_.tail[1] = FRAME_TAIL[1];

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[SentrySerial] 发送失败: {}", e.what());
  }
}

void SentrySerial::read_thread()
{
  tools::logger()->info("[SentrySerial] 读线程启动");

  uint8_t buffer[SENTRY_RX_SIZE];
  int error_count = 0;
  int rx_count = 0;

  while (!quit_) {
    if (!serial_.isOpen()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(100));
      continue;
    }
    if (error_count > 1000) {
      error_count = 0;
      tools::logger()->warn("[SentrySerial] 连续错误过多，尝试重连");
      reconnect();
      continue;
    }

    // ── 逐字节找帧头 "SP"（下位机可能从任意位置开始发）──
    uint8_t byte = 0;
    try {
      if (serial_.read(&byte, 1) != 1) {
        ++error_count;
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        continue;
      }
    } catch (...) {
      ++error_count;
      continue;
    }
    if (byte != FRAME_HEAD[0]) continue;

    try {
      if (serial_.read(&byte, 1) != 1) {
        ++error_count;
        continue;
      }
    } catch (...) {
      ++error_count;
      continue;
    }
    if (byte != FRAME_HEAD[1]) continue;

    const auto timestamp = std::chrono::steady_clock::now();
    buffer[0] = FRAME_HEAD[0];
    buffer[1] = FRAME_HEAD[1];

    try {
      if (serial_.read(buffer + 2, SENTRY_RX_SIZE - 2) != SENTRY_RX_SIZE - 2) {
        ++error_count;
        continue;
      }
    } catch (...) {
      ++error_count;
      continue;
    }

    // ── 校验帧尾（⚠️ 哨兵协议用固定尾，不是 CRC16）──
    const auto * pkt = reinterpret_cast<const SentryToVisionPacket *>(buffer);
    if (pkt->tail[0] != FRAME_TAIL[0] || pkt->tail[1] != FRAME_TAIL[1]) {
      tools::logger()->debug(
        "[SentrySerial] 帧尾错误: {:02X}{:02X}（应为 BB66）", pkt->tail[0], pkt->tail[1]);
      ++error_count;
      continue;
    }

    error_count = 0;
    ++rx_count;

    const Eigen::Quaterniond q(pkt->q[0], pkt->q[1], pkt->q[2], pkt->q[3]);
    queue_.push({q, timestamp});

    {
      std::lock_guard<std::mutex> lk(mutex_);
      rx_data_ = *pkt;
      latest_q_ = q;
      state_.yaw = pkt->yaw;
      state_.yaw_vel = pkt->yaw_vel;
      state_.pitch = pkt->pitch;
      state_.pitch_vel = pkt->pitch_vel;
      state_.bullet_speed = pkt->bullet_speed;
      state_.bullet_count = pkt->bullet_count;
      state_.health = pkt->health;
      state_.game_state = pkt->game_state;
      switch (pkt->mode) {
        case 0: mode_ = SentryMode::IDLE; break;
        case 1: mode_ = SentryMode::AUTO_AIM; break;
        case 2: mode_ = SentryMode::SMALL_BUFF; break;
        case 3: mode_ = SentryMode::BIG_BUFF; break;
        default: mode_ = SentryMode::IDLE; break;
      }
    }

    // 限频：每 30 包一条（约 1 秒 @30Hz）
    if (rx_count % 30 == 1)
      tools::logger()->debug(
        "[SentrySerial] RX #{}: mode={} game={} yaw={:.1f}° pitch={:.1f}° hp={}", rx_count,
        pkt->mode, pkt->game_state, pkt->yaw * 57.3, pkt->pitch * 57.3, pkt->health);
  }

  tools::logger()->info("[SentrySerial] 读线程结束");
}

void SentrySerial::reconnect()
{
  connected_ = false;
  for (int i = 0; i < 10 && !quit_; ++i) {
    try {
      if (serial_.isOpen()) serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
      serial_.open();
      connected_ = true;
      queue_.clear();
      tools::logger()->info("[SentrySerial] 重连成功（第 {} 次尝试）", i + 1);
      return;
    } catch (const std::exception & e) {
      tools::logger()->warn("[SentrySerial] 重连 {}/10 失败: {}", i + 1, e.what());
    }
  }
  tools::logger()->error("[SentrySerial] 重连 10 次均失败");
}

}  // namespace io
