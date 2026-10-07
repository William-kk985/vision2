/**
 * @file  io/board/sentry_serial/sentry_serial.hpp
 * @brief 哨兵串口通信（协议 + 驱动）
 *
 * ## 为什么哨兵要单独一份
 * 步兵/英雄走 `io::Gimbal`（`io/board/gimbal/`），其上行/下行用 **CRC16** 作校验；
 * 而哨兵与下位机之间用的是**固定帧尾 `0xBB 0x66`**，且**多了若干字段**
 * （上行 `health` / `game_state`，下行 `nav_x/y/w` / `status_position`）。
 * 两者字段集与校验方式都不同，故单列一份，**不与 `io::Gimbal` 合并**。
 *
 * ## 协议（与下位机侧 Python 实现逐字节一致）
 * ```
 * 帧头 "SP" (0x53 0x50)        帧尾 0xBB 0x66
 * ```
 *
 * ### 上行 `SentryToVisionPacket`（46 字节，下位机 → 上位机）
 * ```
 * 偏移   字段            类型       长度  说明
 * 0x00   head[2]         uint8[2]   2     "SP"
 * 0x02   mode            uint8      1     0:idle 1:auto_aim 2:small_buff 3:big_buff
 * 0x03   q[4]            float[4]   16    四元数，wxyz 顺序
 * 0x13   yaw             float      4     rad
 * 0x17   yaw_vel         float      4     rad/s
 * 0x1B   pitch           float      4     rad
 * 0x1F   pitch_vel       float      4     rad/s
 * 0x23   bullet_speed    float      4     m/s
 * 0x27   bullet_count    uint16     2
 * 0x29   health          uint16     2     ⭐ 本项目 `io::Gimbal` 无此字段
 * 0x2B   game_state      uint8      1     0:game_end 1:game_start  ⭐ 同上
 * 0x2C   tail[2]         uint8[2]   2     0xBB 0x66
 * ```
 * 合计 2+1+16+4+4+4+4+4+2+2+1+2 = **46 字节**
 * （Python `struct` 格式串 `<2sB9fHHB2s`）
 *
 * ### 下行 `VisionToSentryPacket`（42 字节，上位机 → 下位机）
 * ```
 * 偏移   字段              类型       长度  说明
 * 0x00   head[2]           uint8[2]   2     "SP"
 * 0x02   mode              uint8      1     0:no_ctrl 1:ctrl 2:ctrl+fire
 * 0x03   yaw               float      4     rad
 * 0x07   yaw_vel           float      4     rad/s
 * 0x0B   yaw_acc           float      4     rad/s^2
 * 0x0F   pitch             float      4     rad
 * 0x13   pitch_vel         float      4     rad/s
 * 0x17   pitch_acc         float      4     rad/s^2
 * 0x1B   nav_x             float      4     m      ⭐ `io::Gimbal` 无此字段
 * 0x1F   nav_y             float      4     m      ⭐ 同上
 * 0x23   nav_w             float      4     rad    ⭐ 同上
 * 0x27   status_position   uint8      1     ⭐ 见下方"无导航时要发什么"
 * 0x28   tail[2]           uint8[2]   2     0xBB 0x66
 * ```
 *
 * ## ⭐⭐ 导航数据可缺省 —— 没有导航时这一帧【照发】
 *
 * ⭐ **本帧与导航【无关】的字段永远有效**：`mode` / `yaw{,_vel,_acc}` / `pitch{,_vel,_acc}`
 *   ⇒ **云台控制不依赖导航包**，`nav_*` 缺失时它们照常下发（这是哨兵能独立工作的前提）。
 *
 * | 场景 | `nav_x` / `nav_y` / `nav_w` | `status_position` |
 * |---|---|---|
 * | ⭐ **导航包未移植/未就绪** | ⭐ **全部 0** | ⭐ **用它发"别的信息"**（见下） |
 * | ⭐ **导航包已就绪** | 填入真实导航数据 | 按需（默认 0） |
 *
 * ### `status_position` 的语义（无导航时的"别的信息"通道）
 * 它是 1 字节的**通用状态位**，由视觉侧填写、下位机解读。⚠️ **取值需与下位机约定**。
 * 建议编码（⚠️ 尚未与下位机确认，接入前必须对齐）：
 * ```
 * bit0-1  跟踪状态    0=lost 1=detecting 2=tracking 3=temp_lost
 * bit2    是否可开火  0=不可 1=可（= Plan::fire）
 * bit3-6  目标编号    ArmorName（0..8），无目标时 0
 * bit7    保留
 * ```
 * ⚠️ **这样在没有导航的过渡期，下位机仍能从这一帧读到"打到谁/能不能打"**，
 *    而不必等导航包移植完成。
 * 📌 **待办**：与下位机确认 `status_position` 的编码后再启用；在此之前保持 0。
 * 合计 2+1+36+1+2 = **42 字节**（Python `<2sB9fB2s`）
 *
 * ## ⚠️ 与参考实现（旧版赫兹 `src/drivers/sentry_serial/`）的差异
 * | 项 | 参考实现 | 本实现 | 理由 |
 * |---|---|---|---|
 * | 帧格式 | 逐字节相同 | ⭐ **逐字节相同** | 必须与下位机一致，不可改 |
 * | 波特率 | 115200 | 115200 | 同 |
 * | yaw/pitch 偏置 | ⚠️ **硬编码 −1.8° / −6.6°** | ⭐ **读 yaml `yaw_offset` / `pitch_offset`** | 标定值不应埋在代码里；本项目既有配置项 |
 * | 串口缺失 | ⚠️ 仅报 error，`connected_=false` | ⭐ 报**可操作的**错误（列出机器上可用串口）+ 可选严格模式 | 沿用本项目 W53 的做法 |
 * | 日志 | `utils/base/logger.hpp`（旧路径） | `utils/log/logger.hpp` | 本项目路径 |
 *
 * ## ⚠️ 当前状态：**仅提供协议与驱动，尚未接入 `sentry.cpp`**
 * 哨兵主循环**仍使用 `io::CBoard`（CAN）**。接口替换（含 `q(t)` / `state()` 的适配、
 * 四元数来源、导航数据的下行方式）待后续单独处理。
 *
 * @see io/board/gimbal/gimbal.hpp  —— 步兵/英雄的串口实现（CRC16 校验）
 * @see io/board/can/cboard.hpp     —— 哨兵/无人机当前使用的 CAN 实现
 */
#ifndef HZMIR_IO_BOARD_SENTRY_SERIAL_SENTRY_SERIAL_HPP
#define HZMIR_IO_BOARD_SENTRY_SERIAL_SENTRY_SERIAL_HPP

#include <Eigen/Geometry>
#include <atomic>
#include <chrono>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include <tuple>

#include "serial/serial.h"
#include "utils/concurrency/thread_safe_queue.hpp"

namespace io
{

/// 帧头 `"SP"`
constexpr uint8_t FRAME_HEAD[2] = {0x53, 0x50};
/// 帧尾 `0xBB 0x66`（⚠️ 不是 CRC16 —— 与 `io::Gimbal` 的区别）
constexpr uint8_t FRAME_TAIL[2] = {0xBB, 0x66};

/// 上行包长度（下位机 → 上位机）
constexpr std::size_t SENTRY_RX_SIZE = 46;
/// 下行包长度（上位机 → 下位机）
constexpr std::size_t SENTRY_TX_SIZE = 42;

#pragma pack(push, 1)

/// @brief 上行：下位机 → 上位机（46 字节）。布局见文件头注释。
struct SentryToVisionPacket
{
  uint8_t head[2];        // "SP"
  uint8_t mode;           // 0:idle 1:auto_aim 2:small_buff 3:big_buff
  float q[4];             // 四元数 wxyz
  float yaw;              // rad
  float yaw_vel;          // rad/s
  float pitch;            // rad
  float pitch_vel;        // rad/s
  float bullet_speed;     // m/s
  uint16_t bullet_count;
  uint16_t health;        // ⭐ 本项目 io::Gimbal 无此字段
  uint8_t game_state;     // 0:game_end 1:game_start
  uint8_t tail[2];        // 0xBB 0x66
};

/// @brief 下行：上位机 → 下位机（42 字节）。布局见文件头注释。
struct VisionToSentryPacket
{
  uint8_t head[2];         // "SP"
  uint8_t mode;            // 0:no_ctrl 1:ctrl 2:ctrl+fire
  float yaw;               // rad
  float yaw_vel;           // rad/s
  float yaw_acc;           // rad/s^2
  float pitch;             // rad
  float pitch_vel;         // rad/s
  float pitch_acc;         // rad/s^2
  float nav_x;             // m     ⭐ io::Gimbal 无此字段
  float nav_y;             // m     ⭐ 同上
  float nav_w;             // rad   ⭐ 同上
  uint8_t status_position; // 预留
  uint8_t tail[2];         // 0xBB 0x66
};

#pragma pack(pop)

// ⭐ 长度必须与下位机一致 —— 编译期卡死（这是整份协议的契约）
static_assert(
  sizeof(SentryToVisionPacket) == SENTRY_RX_SIZE, "SentryToVisionPacket 必须是 46 字节");
static_assert(
  sizeof(VisionToSentryPacket) == SENTRY_TX_SIZE, "VisionToSentryPacket 必须是 42 字节");

/// @brief 下位机上报的工作模式（与 `io::GimbalMode` 分开：字段布局不同）
enum class SentryMode { IDLE, AUTO_AIM, SMALL_BUFF, BIG_BUFF };

/// @brief 下位机状态
struct SentryState
{
  float yaw = 0;
  float yaw_vel = 0;
  float pitch = 0;
  float pitch_vel = 0;
  float bullet_speed = 27.0f;
  uint16_t bullet_count = 0;
  uint16_t health = 0;        // ⭐ 仅哨兵串口协议有
  uint8_t game_state = 0;     // ⭐ 同上
};

/// @brief 哨兵串口驱动。
///
/// ⚠️ **尚未接入 `sentry.cpp`** —— 哨兵主循环当前仍用 `io::CBoard`（CAN）。
/// 本类先提供协议与收发能力，供接口替换时使用。
class SentrySerial
{
public:
  explicit SentrySerial(const std::string & config_path);
  ~SentrySerial();

  SentrySerial(const SentrySerial &) = delete;
  SentrySerial & operator=(const SentrySerial &) = delete;

  /// @brief 下位机当前模式
  SentryMode mode() const;

  /// @brief 下位机当前状态
  SentryState state() const;

  /// @brief 取 `t` 时刻的云台姿态（队列内 slerp 插值；队列空则返回最近值）
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point t);

  /// @brief 最近一次收到的姿态（不加时间戳对齐）
  Eigen::Quaterniond latest_q() const;

  /// @brief 是否已连上串口
  bool is_connected() const { return connected_.load(); }

  /// @brief 最近一次收到的上行包（诊断用）
  SentryToVisionPacket last_rx_packet() const;

  /// @brief 下发一帧（⭐ **`nav_*` 可缺省 —— 没有导航数据时传 0 即可，本帧照发**）
  ///
  /// @note `control`/`fire` 编码为 `mode`：`!control → 0`，`control && !fire → 1`，
  ///       `control && fire → 2`。`yaw`/`pitch` 会**加上 yaml 里的偏置**。
  /// @note ⭐ **导航缺省不影响云台控制**：`nav_*` 与 `status_position` 默认 0，
  ///       而 `mode` / `yaw*` / `pitch*` 始终按传入值下发。
  ///       ⇒ 哨兵在**导航包移植完成前**即可正常工作。
  /// @param nav_x 导航 X（m）。⚠️ 无导航时传 0
  /// @param nav_y 导航 Y（m）。⚠️ 同上
  /// @param nav_w 导航朝向（rad）。⚠️ 同上
  /// @param status_position ⭐ **无导航时的"别的信息"通道**（1 字节，语义见文件头
  ///        "无导航时要发什么"）。⚠️ 编码需与下位机约定；未约定前保持 0。
  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc, float nav_x = 0, float nav_y = 0, float nav_w = 0,
    uint8_t status_position = 0);

  /// @brief 模式名（日志用）
  static const char * mode_str(SentryMode m);

private:
  void read_thread();
  void reconnect();

  serial::Serial serial_;
  std::string port_name_;

  std::thread thread_;
  std::atomic<bool> quit_{false};
  std::atomic<bool> connected_{false};
  mutable std::mutex mutex_;

  SentryToVisionPacket rx_data_{};
  VisionToSentryPacket tx_data_{};

  SentryMode mode_ = SentryMode::IDLE;
  SentryState state_;
  Eigen::Quaterniond latest_q_ = Eigen::Quaterniond::Identity();

  /// ⭐ 偏置（从 yaml 读，**不硬编码** —— 参考实现里写死 −1.8°/−6.6°）
  float yaw_offset_ = 0;
  float pitch_offset_ = 0;

  tools::ThreadSafeQueue<
    std::tuple<Eigen::Quaterniond, std::chrono::steady_clock::time_point>>
    queue_{1000};
};

}  // namespace io

#endif  // HZMIR_IO_BOARD_SENTRY_SERIAL_SENTRY_SERIAL_HPP
