/**
 * @file core/auto_aim/detector/det_stats.hpp
 * @brief ⭐ **检测统计日志的开关**（W63；W98 剥离了统计本体）
 *
 * ## 为什么需要
 * W62 给 YOLO 加了「逐帧候选 → 各步过滤」统计（非常好用 —— 实测直接区分出了
 * 「objectness 没检出」vs「被 not_armor 滤掉」vs「正常输出」三种情况）。
 * ⚠️ 但连续运行时它每 0.3~0.6 秒刷一行，**赛场/长时间跑批时是噪音**。
 *
 * ## 两层控制
 * | 方式 | 效果 |
 * |---|---|
 * | ⭐ **`--det-stats=false`** | **硬关**（不受日志级别影响） |
 * | **日志级别** | 统计走 `debug` 级 → `HZMIR_LOG_LEVEL=info` 或按 `d` 循环即可静音 |
 * | ⭐ **`--log-off=yolov5`** | 按模块静音（W95，运行期热键 `n` 也可） |
 *
 * ⭐ 默认 **开**（`debug` 级别下可见）—— 因为它对调试**很有价值**。
 *
 * ## ⚠️ W98 的变更：统计本体已移出本文件
 * 原来这里还有个**全局旁路** `DetectStats` + `set_last_detect_stats()` /
 * `last_detect_stats()`：
 * ```
 * // ❌ 旧：YOLO 写全局 → 主循环回头读
 * set_last_detect_stats(st);                    // YOLO 内部
 * const auto & st = last_detect_stats();        // 主循环
 * fd.detector.nms_survivors = st.nms_survivors;
 * ```
 * ⚠️ 这条旁路正是 **`best_confidence` / `nms_survivors` 长期为 0**（W64 事故）的温床 ——
 *   它不在类型系统里，忘了填**编译器不会报错**。
 *
 * ⇒ 现在统计**随 `DetectorResult::dbg` 一起返回**（见 `detector_debug.hpp`），
 *   本文件只保留「要不要打这些日志」这一个开关。
 */
#ifndef HZMIR_CORE_AUTO_AIM_DETECTOR_DET_STATS_HPP
#define HZMIR_CORE_AUTO_AIM_DETECTOR_DET_STATS_HPP

#include <atomic>

namespace auto_aim
{

namespace detail
{
inline std::atomic<bool> & det_stats_flag()
{
  static std::atomic<bool> on{true};   // ⭐ 默认开
  return on;
}
}  // namespace detail

/// @brief 设置是否输出检测统计（`--det-stats=false` 关闭）
inline void set_det_stats_enabled(bool on) noexcept
{
  detail::det_stats_flag().store(on, std::memory_order_release);
}

/// @brief 检测统计当前是否启用
inline bool det_stats_enabled() noexcept
{
  return detail::det_stats_flag().load(std::memory_order_acquire);
}

}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_DETECTOR_DET_STATS_HPP
