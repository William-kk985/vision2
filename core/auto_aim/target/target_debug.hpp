/**
 * @file core/auto_aim/target/target_debug.hpp
 * @brief ⭐⭐ 估计器 的 Debug 快照 —— **由该角色自己拥有**（W50 方案 A）
 *
 * ## 为什么放在角色自己的目录（而不是 `core/debug.hpp`）
 * 原来所有角色的 Debug 结构集中在 `core/debug.hpp` → **依赖方向反了**：
 * 「估计器该暴露什么」由**上层契约**决定，而由角色代码来填。
 *
 * ⭐ 后果（W8 加 `Plan` 的 10 个内部量时实测）：**要改 5 个地方**，
 *   其中 3 处在别的目录/语言（`core/debug.hpp` / `utils/debug/csv_sink.cpp` / `scripts/*.py`）。
 *
 * 现在：**加字段只改自己目录**（本文件 + 角色的 `.cpp`）。
 *
 * ## ⚠️ 依赖方向（单向，无环）
 * ```
 * core/auto_aim/target/target_debug.hpp   →  core/debug.hpp（聚合）  →  FrameDebug
 * ```
 * 本文件是**纯数据**，只依赖 `<cstdint>`，**绝不 include `core/debug.hpp`**。
 *
 * ## ⭐ 已完成（W98）
 * 「把填 Debug 塞进角色的返回路径」**已经做了** ——
 * 角色返回 `XxxResult { 业务结果; XxxDebug dbg; }`，
 * 主循环一行 `fd.xxx = r.dbg;`。**不填就编译不过** ⇒ 结构上不可能漏填。
 * 详见 `detector.hpp` 里 `DetectorResult` 的长注释。
 */
#ifndef HZMIR_CORE_AUTO_AIM_TARGET_TARGET_DEBUG_HPP
#define HZMIR_CORE_AUTO_AIM_TARGET_TARGET_DEBUG_HPP

#include <cstdint>

namespace auto_aim
{

/// 估计器
struct TargetDebug
{
  int tracked_id = 0;
  double xyz_world[3] = {0, 0, 0};
  double yaw = 0, w = 0, r = 0, l = 0, h = 0;
  double nis = 0, nis_thresh = 0;   // ⭐ 滤波器健康度
  bool invincible = false;          // ⭐ W7
  int64_t t_update_us = 0;
};

}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_TARGET_TARGET_DEBUG_HPP
