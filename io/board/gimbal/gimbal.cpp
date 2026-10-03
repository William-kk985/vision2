#include "gimbal.hpp"

#include "utils/math/crc.hpp"
#include "utils/log/logger.hpp"
#include "utils/math/math_tools.hpp"
#include "utils/yaml/yaml.hpp"

#include <filesystem>
#include <string>
#include <vector>

namespace io
{
Gimbal::Gimbal(const std::string & config_path)
{
  auto yaml = tools::load(config_path);
  auto com_port = tools::read<std::string>(yaml, "com_port");

  try {
    serial_.setPort(com_port);
    serial_.open();
  } catch (const std::exception & e) {
    // ⭐⭐ W53：**把错误变成可操作的**。原来只有一句 `Failed to open serial` +
    //   `exit(1)` —— 用户不知道：① 试的是哪个设备 ② 这台机器有哪些串口 ③ 怎么办。
    tools::logger()->error("[Gimbal] 打不开串口: {}", e.what());
    tools::logger()->error("[Gimbal]  尝试的设备: '{}'（来自 {} 的 `com_port`）", com_port, config_path);

    // ⭐ 列出这台机器上**实际可用**的串口
    std::vector<std::string> avail;
    std::error_code ec;
    for (const auto & e2 : std::filesystem::directory_iterator("/dev", ec)) {
      const auto n = e2.path().filename().string();
      if (n.rfind("ttyUSB", 0) == 0 || n.rfind("ttyACM", 0) == 0 || n.rfind("ttyS", 0) == 0)
        avail.push_back(e2.path().string());
    }
    if (!avail.empty()) {
      std::string joined;
      for (const auto & a : avail) joined += a + "  ";
      tools::logger()->error("[Gimbal]  本机可用串口: {}", joined);
    } else {
      tools::logger()->error("[Gimbal]  本机没有任何 /dev/ttyUSB* / ttyACM* 设备");
    }

    // ⭐ 从配置路径推导兵种名（如 params/robots/hero.yaml → "hero"），避免硬编码
    std::string robot = "infantry";
    {
      const auto stem = std::filesystem::path(config_path).stem().string();
      if (!stem.empty()) robot = stem;
    }

    // ⭐ 给出**具体怎么办**
    tools::logger()->error(
      "[Gimbal]  ── 怎么办 ──\n"
      "   ① 零硬件验证（没接硬件就用这个）:\n"
      "      tools/scripts/run.sh {} --video=录像.avi --force-mode=1\n"
      "   ② 检查 udev 规则有没有把下位机映射成 '{}':\n"
      "      ls -l '{}'  # 不存在就是这个原因\n"
      "   ③ 改参数文件里的 `com_port` 为实际设备（或插上硬件后用 dmesg 确认）:\n"
      "      dmesg | tail -20 | grep -i tty",
      robot, com_port, com_port);
    exit(1);
  }

  thread_ = std::thread(&Gimbal::read_thread, this);

  // ⭐⭐ W83：原来是无超时的 `queue_.pop()` —— 下位机/串口不通时**构造就永久卡死**
  //   （进程既没日志也不退出，看起来像"启动失败"）。⇒ 改带超时 + **说清怎么办**。
  {
    std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point> first;
    if (!queue_.pop_for(first, std::chrono::milliseconds(3000))) {
      tools::logger()->error(
        "[Gimbal] 3 秒内没收到任何 IMU 数据 → 无法确定云台姿态，启动中止。\n"
        "  排查：① 下位机是否上电、是否在发数据\n"
        "     ② 波特率/协议是否与下位机一致\n"
        "     ③ 只想看检测/跟踪：用 `--no-board` 跑虚拟下位机（IMU 恒为单位四元数）");
      throw std::runtime_error("[Gimbal] 没有 IMU 数据");
    }
  }
  tools::logger()->info("[Gimbal] First q received.");
}

Gimbal::~Gimbal()
{
  quit_ = true;
  if (thread_.joinable()) thread_.join();
  serial_.close();
}

GimbalMode Gimbal::mode() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return mode_;
}

GimbalState Gimbal::state() const
{
  std::lock_guard<std::mutex> lock(mutex_);
  return state_;
}

std::string Gimbal::str(GimbalMode mode) const { return str_of(mode); }

// ⭐ 静态版：不依赖实例（ReplayBoard 等也能用）
std::string Gimbal::str_of(GimbalMode mode)
{
  switch (mode) {
    case GimbalMode::IDLE:       return "IDLE";
    case GimbalMode::AUTO_AIM:   return "AUTO_AIM";
    case GimbalMode::SMALL_BUFF: return "SMALL_BUFF";
    case GimbalMode::BIG_BUFF:   return "BIG_BUFF";
    default:                     return "INVALID";
  }
}

Eigen::Quaterniond Gimbal::q(std::chrono::steady_clock::time_point t)
{
  // ⭐⭐ W83 修复：原来两个调用 **都会无限阻塞**：
  //   · `queue_.pop()`   —— 队列空就永久等
  //   · `queue_.front()` —— 同理
  //   ⚠️ 后果：串口一断，**主循环（每帧都调 q()）永久卡死** →
  //      Ctrl-C / 热键 / `q` 全部失效，只能 `kill -9`。
  //   ⇒ 现在：`pop_for` 带超时；`try_peek` 非阻塞。
  //      超时 → 返回 `last_q_`（上一次有效姿态）+ **限频告警**，主循环继续跑。
  while (true) {
    std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point> item;
    if (!queue_.pop_for(item, std::chrono::milliseconds(200))) {
      const auto now_ms = std::chrono::duration_cast<std::chrono::milliseconds>(
                            std::chrono::steady_clock::now().time_since_epoch()).count();
      const auto last = imu_gap_warned_.load(std::memory_order_relaxed);
      if (now_ms - last > 1000) {   // 每 ~1 秒告警一次（不刷屏）
        imu_gap_warned_.store(now_ms, std::memory_order_relaxed);
        tools::logger()->warn(
          "[Gimbal] IMU 数据断流 >200ms → 本帧姿态沿用上一次（EKF 预测不可信）。"
          "检查下位机/串口；只想看检测可用 `--no-board`");
      }
      return last_q_;
    }
    const auto & [q_a, t_a] = item;
    if (q_a.coeffs().allFinite()) last_q_ = q_a;   // ⭐ 只缓存有限值

    // ⭐ 非阻塞 peek：队列里只有这一个 → 直接用上一次姿态（原来会死等下一个）
    std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point> nxt;
    if (!queue_.try_peek(nxt)) return last_q_;

    const auto & [q_b, t_b] = nxt;
    auto t_ab = tools::delta_time(t_a, t_b);
    auto t_ac = tools::delta_time(t_a, t);
    if (!(t_ab > 0)) continue;                     // ⭐ 防除零（时间戳相同/倒序）
    auto k = t_ac / t_ab;
    Eigen::Quaterniond q_c = q_a.slerp(k, q_b).normalized();
    if (t < t_a) return q_c;
    if (!(t_a < t && t <= t_b)) continue;

    return q_c;
  }
}

void Gimbal::send(io::VisionToGimbal VisionToGimbal)
{
  tx_data_.mode = VisionToGimbal.mode;
  tx_data_.yaw = VisionToGimbal.yaw;
  tx_data_.yaw_vel = VisionToGimbal.yaw_vel;
  tx_data_.yaw_acc = VisionToGimbal.yaw_acc;
  tx_data_.pitch = VisionToGimbal.pitch;
  tx_data_.pitch_vel = VisionToGimbal.pitch_vel;
  tx_data_.pitch_acc = VisionToGimbal.pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

void Gimbal::send(
  bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
  float pitch_acc)
{
  tx_data_.mode = control ? (fire ? 2 : 1) : 0;
  tx_data_.yaw = yaw;
  tx_data_.yaw_vel = yaw_vel;
  tx_data_.yaw_acc = yaw_acc;
  tx_data_.pitch = pitch;
  tx_data_.pitch_vel = pitch_vel;
  tx_data_.pitch_acc = pitch_acc;
  tx_data_.crc16 = tools::get_crc16(
    reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_) - sizeof(tx_data_.crc16));

  try {
    serial_.write(reinterpret_cast<uint8_t *>(&tx_data_), sizeof(tx_data_));
  } catch (const std::exception & e) {
    tools::logger()->warn("[Gimbal] Failed to write serial: {}", e.what());
  }
}

bool Gimbal::read(uint8_t * buffer, size_t size)
{
  try {
    return serial_.read(buffer, size) == size;
  } catch (const std::exception & e) {
    // tools::logger()->warn("[Gimbal] Failed to read serial: {}", e.what());
    return false;
  }
}

void Gimbal::read_thread()
{
  tools::logger()->info("[Gimbal] read_thread started.");
  int error_count = 0;

  while (!quit_) {
    if (error_count > 5000) {
      error_count = 0;
      tools::logger()->warn("[Gimbal] Too many errors, attempting to reconnect...");
      reconnect();
      continue;
    }

    if (!read(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (rx_data_.head[0] != 'S' || rx_data_.head[1] != 'P') continue;

    auto t = std::chrono::steady_clock::now();

    if (!read(
          reinterpret_cast<uint8_t *>(&rx_data_) + sizeof(rx_data_.head),
          sizeof(rx_data_) - sizeof(rx_data_.head))) {
      error_count++;
      continue;
    }

    if (!tools::check_crc16(reinterpret_cast<uint8_t *>(&rx_data_), sizeof(rx_data_))) {
      tools::logger()->debug("[Gimbal] CRC16 check failed.");
      continue;
    }

    error_count = 0;
    Eigen::Quaterniond q(rx_data_.q[0], rx_data_.q[1], rx_data_.q[2], rx_data_.q[3]);
    queue_.push({q, t});

    std::lock_guard<std::mutex> lock(mutex_);

    state_.yaw = rx_data_.yaw;
    state_.yaw_vel = rx_data_.yaw_vel;
    state_.pitch = rx_data_.pitch;
    state_.pitch_vel = rx_data_.pitch_vel;
    state_.bullet_speed = rx_data_.bullet_speed;
    state_.bullet_count = rx_data_.bullet_count;

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

void Gimbal::reconnect()
{
  int max_retry_count = 10;
  for (int i = 0; i < max_retry_count && !quit_; ++i) {
    tools::logger()->warn("[Gimbal] Reconnecting serial, attempt {}/{}...", i + 1, max_retry_count);
    try {
      serial_.close();
      std::this_thread::sleep_for(std::chrono::seconds(1));
    } catch (...) {
    }

    try {
      serial_.open();  // 尝试重新打开
      queue_.clear();
      tools::logger()->info("[Gimbal] Reconnected serial successfully.");
      break;
    } catch (const std::exception & e) {
      tools::logger()->warn("[Gimbal] Reconnect failed: {}", e.what());
      std::this_thread::sleep_for(std::chrono::seconds(1));
    }
  }
}

}  // namespace io