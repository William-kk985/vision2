/**
 * @file core/auto_aim/detector/detector_debug.hpp
 * @brief ⭐⭐ 识别器 的 Debug 快照 —— **由该角色自己拥有**（W50 方案 A）
 *
 * ## 为什么放在角色自己的目录（而不是 `core/debug.hpp`）
 * 原来所有角色的 Debug 结构集中在 `core/debug.hpp` → **依赖方向反了**：
 * 「识别器该暴露什么」由**上层契约**决定，而由角色代码来填。
 *
 * ⭐ 后果（W8 加 `Plan` 的 10 个内部量时实测）：**要改 5 个地方**，
 *   其中 3 处在别的目录/语言（`core/debug.hpp` / `utils/debug/csv_sink.cpp` / `scripts/*.py`）。
 *
 * 现在：**加字段只改自己目录**（本文件 + 角色的 `.cpp`）。
 *
 * ## ⚠️ 依赖方向（单向，无环）
 * ```
 * core/auto_aim/detector/detector_debug.hpp   →  core/debug.hpp（聚合）  →  FrameDebug
 * ```
 * 本文件是**纯数据**，只依赖 `<cstdint>`，**绝不 include `core/debug.hpp`**。
 *
 * ## ⭐ 已完成（W98）
 * 「把填 Debug 塞进角色的返回路径」**已经做了** ——
 * 角色返回 `XxxResult { 业务结果; XxxDebug dbg; }`，
 * 主循环一行 `fd.xxx = r.dbg;`。**不填就编译不过** ⇒ 结构上不可能漏填。
 * 详见 `detector.hpp` 里 `DetectorResult` 的长注释。
 */
#ifndef HZMIR_CORE_AUTO_AIM_DETECTOR_DETECTOR_DEBUG_HPP
#define HZMIR_CORE_AUTO_AIM_DETECTOR_DETECTOR_DEBUG_HPP

#include <cstdint>

namespace auto_aim
{

/// 识别器
struct DetectorDebug
{
  int armor_count = 0;
  double best_confidence = 0;
  int nms_survivors = 0;
    int n_pass = 0;            ///< 过 objectness 门槛的候选数（YOLO 系填）
  int64_t t_infer_us = 0;   // ⭐ L0 本段耗时（学哈工程 DebugExpense）
};

}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_DETECTOR_DETECTOR_DEBUG_HPP
