/**
 * @file core/auto_aim/tracker/tracker_debug.hpp
 * @brief ⭐⭐ 跟踪器 的 Debug 快照 —— **由该角色自己拥有**（W50 方案 A）
 *
 * ## 为什么放在角色自己的目录（而不是 `core/debug.hpp`）
 * 原来所有角色的 Debug 结构集中在 `core/debug.hpp` → **依赖方向反了**：
 * 「跟踪器该暴露什么」由**上层契约**决定，而由角色代码来填。
 *
 * ⭐ 后果（W8 加 `Plan` 的 10 个内部量时实测）：**要改 5 个地方**，
 *   其中 3 处在别的目录/语言（`core/debug.hpp` / `utils/debug/csv_sink.cpp` / `scripts/*.py`）。
 *
 * 现在：**加字段只改自己目录**（本文件 + 角色的 `.cpp`）。
 *
 * ## ⚠️ 依赖方向（单向，无环）
 * ```
 * core/auto_aim/tracker/tracker_debug.hpp   →  core/debug.hpp（聚合）  →  FrameDebug
 * ```
 * 本文件是**纯数据**，只依赖 `<cstdint>`，**绝不 include `core/debug.hpp`**。
 *
 * ## ⭐ 已完成（W98）
 * 「把填 Debug 塞进角色的返回路径」**已经做了** ——
 * 角色返回 `XxxResult { 业务结果; XxxDebug dbg; }`，
 * 主循环一行 `fd.xxx = r.dbg;`。**不填就编译不过** ⇒ 结构上不可能漏填。
 * 详见 `detector.hpp` 里 `DetectorResult` 的长注释。
 */
#ifndef HZMIR_CORE_AUTO_AIM_TRACKER_TRACKER_DEBUG_HPP
#define HZMIR_CORE_AUTO_AIM_TRACKER_TRACKER_DEBUG_HPP

#include <cstdint>

namespace auto_aim
{

/// 跟踪器
struct TrackerDebug
{
  int state = 0;            // 0=lost 1=detecting 2=tracking 3=temp_lost
  int armor_count = 0;
  int priority_mode = 0;    // ⭐ W7
  int filtered_out = 0;     // ⭐ W7 被射击过滤器滤掉几块
  int invincible_count = 0; // ⭐ W35 ROS2 下行：当前已知无敌敌人个数
  int focus_target_count = 0;  // ⭐ W35 ROS2 下行：集火指令指定了几个目标
  int64_t t_track_us = 0;
};

}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_TRACKER_TRACKER_DEBUG_HPP
