/**
 * @file io/board/virtual_board.hpp
 * @brief ⭐⭐⭐ **无下位机板子** —— 只有摄像头、没接串口时用
 *
 * ## 用户场景（W54）
 * 「**这台电脑只插了摄像头，想要尝试一下**」——
 * 原来没串口就 `exit(1)`，摄像头明明能用却跑不起来。
 *
 * ## 它做什么
 * 实现 `io::IBoard`，但**不碰任何硬件**：
 * | 接口 | 返回 |
 * |---|---|
 * | `mode()` | 构造时指定的档位（或 `--force-mode`） |
 * | `state()` | 全 0（`bullet_speed` 用配置值） |
 * | `q(t)` | **单位四元数**（即"云台正对前方"） |
 * | `send(...)` | ⭐ **计数 + 前几次 debug 打印，然后丢弃**（没有电机会响应） |
 *
 * ## ⚠️ 它**不能**替代真下位机的地方（必须说清楚）
 * | 能力 | 有它吗 |
 * |---|---|
 * | 你**能**看检测/跟踪/解算/规划的内部量（CSV / PlotJuggler / 窗口） | ✅ |
 * | 你**能**验证主链路不崩、耗时分布 | ✅ |
 * | ⚠️ **IMU 姿态** —— 恒为单位四元数 | ❌ **拿不到真实云台姿态** |
 * | ⚠️ **弹速** —— 用配置的固定值 | ❌ 拿不到真实弹速 |
 * | ⚠️ **实际弹道 / 命中** | ❌ **无法验证** |
 * | ⚠️ **EKF 的 yaw/ω 预测质量** | ❌ 没有真实 `q`，预测结果**不可信** |
 *
 * ⭐ **所以它适合"看主链路 + 调检测/跟踪参数"，不适合"调弹道/命中"。**
 *
 * ## ⭐ 自动降级（用户要的"感应"）
 * `src/*.cpp` 里：如果配置的 `com_port` **不存在** →
 * ⚠️ **大声警告 + 自动改用 `VirtualBoard`**（而不是 `exit(1)`）。
 * 要恢复"没下位机就失败"的同济行为 → `--strict-board`。
 */
#ifndef HZMIR_IO_BOARD_VIRTUAL_BOARD_HPP
#define HZMIR_IO_BOARD_VIRTUAL_BOARD_HPP

#include <atomic>
#include <chrono>
#include <string>

#include <Eigen/Geometry>

#include "io/board/board.hpp"
#include "utils/log/logger.hpp"

namespace io
{

/// @brief 无下位机板子（不碰硬件；`q` 恒为单位四元数）
class VirtualBoard : public IBoard
{
public:
  /// @param mode 上报的档位
  /// @param bullet_speed 上报的弹速（配置值；没有真下位机时不会更新）
  /// @param reason 日志里说明"为什么用了它"（如 "串口 /dev/gimbal 不存在"）
  VirtualBoard(GimbalMode mode = GimbalMode::AUTO_AIM, double bullet_speed = 22.0,
               std::string reason = "未指定原因")
  : mode_(mode), bullet_speed_(static_cast<float>(bullet_speed))
  {
    tools::logger()->warn(
      "[VirtualBoard] ⚠️ **使用虚拟下位机**（{}）\n"
      "    能做什么: 看检测/跟踪/解算/规划的**内部量**（CSV / 窗口 / PlotJuggler）、验证主链路不崩\n"
      "    ⚠️ 不能做什么: **IMU 姿态恒为单位四元数**、弹速固定 {:.1f} m/s\n"
      "       ⇒ **EKF 的 yaw/ω 预测、弹道误差、命中判定都不可信**\n"
      "    想恢复『没下位机就失败』: 加 --strict-board",
      reason, static_cast<double>(bullet_speed_));
  }

  GimbalMode mode() const override { return mode_; }

  GimbalState state() const override
  {
    // ⭐ 全 0 + 配置的弹速（没有真硬件，这些量本来就没有来源）
    return GimbalState{0.F, 0.F, 0.F, 0.F, bullet_speed_, 0};
  }

  /// ⭐ **单位四元数**（云台正对前方）
  Eigen::Quaterniond q(std::chrono::steady_clock::time_point /*t*/) override
  {
    return Eigen::Quaterniond::Identity();
  }

  void send(
    bool control, bool fire, float yaw, float yaw_vel, float yaw_acc, float pitch, float pitch_vel,
    float pitch_acc) override
  {
    (void)yaw_vel; (void)yaw_acc; (void)pitch_vel; (void)pitch_acc;
    const auto n = ++sent_;
    // ⭐ 前 3 次打印（让你确认真在跑），之后静默计数（不刷屏）
    if (n <= 3)
      tools::logger()->info(
        "[VirtualBoard] send#{:<3} control={} shoot={} yaw={:.4f} pitch={:.4f}  （无硬件，已丢弃）",
        n, control, fire, static_cast<double>(yaw), static_cast<double>(pitch));
    else if (n % 500 == 0)
      tools::logger()->debug("[VirtualBoard] 已发送 {} 条指令（全部丢弃）", n);
  }

  void set_mode(GimbalMode m) { mode_ = m; }

  /// @brief 已"发送"的指令数（诊断用）
  int64_t sent_count() const { return sent_.load(); }

private:
  /// ⭐ W66：最新一条指令（供汇总日志打印真实内容）
  struct LastCmd
  {
    bool control = false, fire = false;
    float yaw = 0, yaw_vel = 0, yaw_acc = 0, pitch = 0, pitch_vel = 0, pitch_acc = 0;
  };
  LastCmd last_;

  GimbalMode mode_;
  float bullet_speed_;
  std::atomic<int64_t> sent_{0};
};

}  // namespace io

#endif  // HZMIR_IO_BOARD_VIRTUAL_BOARD_HPP
