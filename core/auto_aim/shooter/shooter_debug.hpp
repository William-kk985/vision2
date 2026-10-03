/**
 * @file core/auto_aim/shooter/shooter_debug.hpp
 * @brief ⭐⭐ 开火器 的 Debug 快照 —— **由该角色自己拥有**（W50 方案 A）
 *
 * ## 为什么放在角色自己的目录（而不是 `core/debug.hpp`）
 * 原来所有角色的 Debug 结构集中在 `core/debug.hpp` → **依赖方向反了**：
 * 「开火器该暴露什么」由**上层契约**决定，而由角色代码来填。
 *
 * ⭐ 后果（W8 加 `Plan` 的 10 个内部量时实测）：**要改 5 个地方**，
 *   其中 3 处在别的目录/语言（`core/debug.hpp` / `utils/debug/csv_sink.cpp` / `scripts/*.py`）。
 *
 * 现在：**加字段只改自己目录**（本文件 + 角色的 `.cpp`）。
 *
 * ## ⚠️ 依赖方向（单向，无环）
 * ```
 * core/auto_aim/shooter/shooter_debug.hpp   →  core/debug.hpp（聚合）  →  FrameDebug
 * ```
 * 本文件是**纯数据**，只依赖 `<cstdint>`，**绝不 include `core/debug.hpp`**。
 *
 * ## ⭐ 下一步可做（未做）
 * 把「填 Debug」塞进角色的**返回路径**（如 `struct XxxResult { Xxx out; XxxDebug dbg; }`）
 * → **结构上不可能忘填**。
 */
#ifndef HZMIR_CORE_AUTO_AIM_SHOOTER_SHOOTER_DEBUG_HPP
#define HZMIR_CORE_AUTO_AIM_SHOOTER_SHOOTER_DEBUG_HPP

#include <cstdint>

namespace auto_aim
{

/// 开火器
struct ShooterDebug
{
  double traj_err_at_fire = 0, fire_thresh = 0;
  bool should_fire = false;
  bool blocked_by_invincible = false;   // ⭐ W7
  bool blocked_by_filter = false;       // ⭐ W7
  int64_t t_since_last_fire_us = 0;

  // ⭐⭐ W98：从 `Shooter::Decision` 并过来的字段。
  //   原来 `shoot()` 只返回 `bool`，W72 额外加了 `Decision` + `last_decision()` 访问器
  //   —— ⚠️ 那是**半个** Result 模式（数据带出来了，但要主循环主动去取，忘了就静默）。
  //   现在 `shoot()` 直接返回本结构，`Decision` 已删。
  //   ⚠️ 另注：这四个字段**只有 `uav` 走 `Shooter` 时才会被填**；
  //      infantry/hero/sentry 走 `Plan::fire`（MPC 自带判据），这些保持 0。
  bool blocked_by_no_target = false;
  bool blocked_by_auto_fire_off = false;
  bool blocked_by_no_control = false;
  double tolerance = 0;                 ///< 本次用的开火容差
};

}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_SHOOTER_SHOOTER_DEBUG_HPP
