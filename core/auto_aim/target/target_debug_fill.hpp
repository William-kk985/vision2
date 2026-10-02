/**
 * @file core/auto_aim/target/target_debug_fill.hpp
 * @brief ⭐⭐ **从 `Target` 填充 `TargetDebug`**（W70）
 *
 * ## 为什么单独一个头文件
 * `target_debug.hpp` 是**零依赖 POD**（只 `#include <cstdint>`）—— 这是本项目的硬纪律：
 * 角色 debug 结构体不依赖业务类型。⚠️ 但"从 `Target` 取值"必然依赖 `Target`。
 * ⇒ 拆开：**POD 保持零依赖**，填充逻辑（有依赖）单独放这里，**只给主程序用**。
 *
 * ## 解决什么问题
 * `TargetDebug` 的 13 个字段**从没被任何主程序赋值**（W67 排查）：
 * ```
 * grep -n "fd\.target\." src/*.cpp   → 零命中
 * ```
 * ⇒ CSV 里 `tgt_id` / `tgt_x/y/z` / `tgt_yaw` / `tgt_w` / `tgt_r/l/h` /
 *   `tgt_nis` / `tgt_nis_thresh` / `tgt_invincible` / `tgt_t_update_us` **13 列永远是 0**。
 * ⭐ 而这些正是**调 EKF 最需要的量**（尤其 `nis` —— 滤波器健康度）。
 *
 * ## EKF 状态布局（11 维，见 `target.cpp` 的注释）
 * ```
 * x[0] x  x[1] vx  x[2] y  x[3] vy  x[4] z  x[5] vz
 * x[6] a(角度)  x[7] w(角速度)  x[8] r(半径)  x[9] l(长)  x[10] h(高)
 * ```
 */
#ifndef HZMIR_CORE_AUTO_AIM_TARGET_TARGET_DEBUG_FILL_HPP
#define HZMIR_CORE_AUTO_AIM_TARGET_TARGET_DEBUG_FILL_HPP

#include "core/auto_aim/target/target.hpp"
#include "core/auto_aim/target/target_debug.hpp"

namespace auto_aim
{

/// @brief 把 `Target` 的 EKF 状态写进 `TargetDebug`
/// @param d            目标（会被就地修改）
/// @param t            已更新的 `Target`
/// @param t_update_us  EKF 更新耗时（µs；由调用方计时，本函数不测）
inline void fill_target_debug(TargetDebug & d, const Target & t, int64_t t_update_us)
{
  const Eigen::VectorXd & x = t.ekf_x();
  // ⚠️ 维度防御：未初始化的 Target 状态可能是空/短向量（`Target() = default`）
  if (x.size() >= 11) {
    d.xyz_world[0] = x[0];   // x
    d.xyz_world[1] = x[2];   // y
    d.xyz_world[2] = x[4];   // z
    d.yaw = x[6];            // 角度 a
    d.w = x[7];              // 角速度 ω
    d.r = x[8];              // 半径 r
    d.l = x[9];              // 长 l
    d.h = x[10];             // 高 h
  }
  d.tracked_id = t.last_id;
  // ⭐ 滤波器健康度：`last_nis` 是 EKF 的 public 成员；阈值取当前全局设置
  d.nis = t.ekf().last_nis;
  d.nis_thresh = tools::ExtendedKalmanFilter::nis_fail_threshold();
  d.t_update_us = t_update_us;
}

}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_TARGET_TARGET_DEBUG_FILL_HPP
