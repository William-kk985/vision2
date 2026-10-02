/**
 * @file core/auto_aim/detector/det_stats.hpp
 * @brief ⭐⭐ **检测统计日志的开关**（W63）
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
 *
 * ⭐ 默认 **开**（`debug` 级别下可见）—— 因为它对调试**很有价值**。
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

/// @brief ⭐⭐ 一帧的检测统计（供 `frame_debug` 填 CSV）
///
/// ## 为什么需要它（W64）
/// `DetectorDebug::best_confidence` 和 `nms_survivors` **两者都从没被赋值过** →
/// ⚠️ CSV 里 `det_best_conf` 和 `det_nms` **两列永远是 0**，误导分析。
/// （`grep -rn nms_survivors` 只命中 test / csv_sink / 定义，**src 里零赋值**）
///
/// ⭐ 这些数只在 **YOLO 内部**（`detect()`）才知道：
/// `n_pass` = objectness 通过的候选数、`nms_survivors` = NMS 存活数、`n_out` = 最终输出。
/// ⇒ 用一个轻量全局通道带出来，主程序再填进 `FrameDebug`。
struct DetectStats
{
  int n_pass = 0;         ///< objectness 通过后的候选数（"像装甲板"的）
  int nms_survivors = 0;  ///< NMS 存活数（过滤前）
  int n_out = 0;          ///< 最终输出（过 check_name/check_type 后）
  double best_conf = 0;   ///< 最高置信度
};

namespace detail
{
inline DetectStats & last_stats_slot()
{
  static DetectStats st;
  return st;
}
}  // namespace detail

/// @brief 由 YOLO 在 `detect()` 末尾调用
inline void set_last_detect_stats(const DetectStats & st) noexcept
{
  detail::last_stats_slot() = st;
}

/// @brief 主程序读取（填 `fd.detector.*`）
inline const DetectStats & last_detect_stats() noexcept
{
  return detail::last_stats_slot();
}

}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_MAIM_DETECTOR_DET_STATS_HPP
