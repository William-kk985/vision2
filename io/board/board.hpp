/**
 * @file io/board/board.hpp
 * @brief ⭐ `IBoard`：下位机统一接口 + 共享类型（`GimbalMode` / `GimbalState`）
 *
 * ## ⭐ W31 收口：`io::Gimbal` **直接实现** `IBoard`
 * 本文件原来是「基础类型在 `gimbal.hpp`、接口在本文件、再加一层 `GimbalBoard` 适配」——
 * 现在把**共享类型搬到这里**，`gimbal.hpp` 反过来 include 本文件，于是：
 * ```
 *   io::Gimbal   : public IBoard   ✅ 直接用（不再需要适配层）
 *   io::ReplayBoard : public IBoard ✅ 录像回放
 *   io::CBoard   : 独立接口（见下）
 * ```
 *
 * ## ⚠️ 为什么 `io::CBoard` **不**实现 `IBoard`
 * `CBoard`（CAN 下位机）的 API 与 `Gimbal`（串口下位机）**本质不同**：
 * | | `io::Gimbal` | `io::CBoard` |
 * |---|---|---|
 * | 下发 | `send(8 个标量)` | `send(io::Command)` |
 * | 姿态 | `q(t)`（四元数队列） | `imu_at(t)` |
 * | 档位 | `mode()`（`GimbalMode`） | 公开成员 `Mode mode` + `ShootMode shoot_mode` |
 *
 * ⇒ 强行统一需要**改写 `CBoard` 的接口** —— 那是**改同济的代码**，不是架构收口。
 *   按「源代码以同济为准」原则：**保持原样**，仅在需要统一视角时用适配器（若将来确有需要）。
 */
#ifndef HZMIR_IO_BOARD_BOARD_HPP
#define HZMIR_IO_BOARD_BOARD_HPP

#include <chrono>
#include <cstdint>
#include <string>

#include <Eigen/Geometry>

namespace io
{

/// 云台档位（⭐ 从 `gimbal.hpp` 搬来 —— 它是 `Gimbal`/`ReplayBoard` 的**共享类型**）
enum class GimbalMode
{
  IDLE,        // 空闲
  AUTO_AIM,    // 自瞄
  SMALL_BUFF,  // 小符
  BIG_BUFF     // 大符
};

/// 云台状态（⭐ 同上）
struct GimbalState
{
  float yaw;
  float yaw_vel;
  float pitch;
  float pitch_vel;
  float bullet_speed;
  uint16_t bullet_count;
};

/// @brief 下位机统一接口
class IBoard
{
public:
  virtual ~IBoard() = default;

  virtual GimbalMode mode() const = 0;
  virtual GimbalState state() const = 0;
  virtual Eigen::Quaterniond q(std::chrono::steady_clock::time_point t) = 0;

  virtual void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc) = 0;
};

}  // namespace io

#endif  // HZMIR_IO_BOARD_BOARD_HPP
