/**
 * @file utils/wheels/track/temporal_integrator.hpp
 * @brief ⭐ 轮子：时序积分（多帧累积增强检测稳定性）—— 电子科技大学
 *
 * **来源**：`Hz_Rm_Vision/src/core/tracking/temporal_integrator.{hpp,cpp}`（174 + 271 行）
 * **落位理由**：算法本身与业务无关，但原实现**硬绑 `Armor`**（4 处字段访问）
 *   → 泛型化后进 `utils/wheels/`，`Armor` 的具体策略由 `auto_aim` 侧特化 `TemporalTraits`
 *
 * ## ⭐ 泛型化顺带修掉的问题
 *
 * | # | 原问题 | 本实现 |
 * |---|---|---|
 * | **E2** | `TIResult{ Armor armor{0, 0, 0.0f, cv::Rect(), {}, cv::Point2f()} }`<br>—— 想用 `0` 构造 `Lightbar`（`Armor` 的第 2 个成员）→ **非法初始化** | ⭐ `TIResult<T>` 里是 `T item{}` → **地雷消失** |
 * | **B1** | `adaptWindowSize(velocity)` 的结果**被丢弃**（`process` 里只用于 `<=0` 判断，<br>没传给 `updateHistoryWindow`）→ **自适应窗口永不生效** | ⭐ velocity 一路传到窗口更新 |
 * | **B2** | `window_size = 0` 时窗口被清空 → `computeConfidence` 里 `match_count / 0` → **NaN** | ⭐ 加 `total_frames <= 0` 保护 |
 *
 * ## 算法
 * ```
 *   ① 维护最近 N 帧检测结果的滑动窗口
 *   ② 对当前帧每个目标，统计它在历史各帧中「同类 + 空间邻近」的出现次数
 *   ③ 置信度 = 出现频率；≥ 阈值才「验证通过」并输出
 *   ④ 自适应：低速用大窗口（多累积），高速直接旁路（不做积分）
 * ```
 *
 * ## 用法
 * ```cpp
 * template <> struct tools::TemporalTraits<auto_aim::Armor> { ... };   // 业务侧特化
 * tools::TemporalIntegrator<auto_aim::Armor> ti(cfg);
 * auto verified = ti.process(armors, t, velocity);
 * ```
 */
#ifndef HZMIR_UTILS_WHEELS_TRACK_TEMPORAL_INTEGRATOR_HPP
#define HZMIR_UTILS_WHEELS_TRACK_TEMPORAL_INTEGRATOR_HPP

#include <algorithm>
#include <cmath>
#include <deque>
#include <list>
#include <opencv2/core.hpp>
#include <string>
#include <unordered_map>
#include <vector>

#include "utils/log/logger.hpp"

namespace tools
{

/// 时序积分配置（原 `auto_aim::TIConfig` 原样）
struct TIConfig
{
  // 基础参数
  int window_size = 3;                 ///< 滑动窗口大小（帧数）
  double confidence_threshold = 0.6;   ///< 置信度阈值 (0-1)

  // 自适应参数
  bool adaptive_mode = true;           ///< 是否启用自适应模式
  double low_speed_threshold = 1.0;    ///< 低速阈值 (m/s)
  double high_speed_threshold = 2.5;   ///< 高速阈值 (m/s)

  // 分级策略参数
  int low_speed_window = 3;            ///< 低速时窗口大小
  int mid_speed_window = 2;            ///< 中速时窗口大小
  bool enable_high_speed = false;      ///< 高速时是否仍启用

  // 空间参数
  double spatial_tolerance = 0.15;     ///< 空间容差（米）
  double min_area = 0.01;              ///< 最小有效面积（平方米）

  // 调试参数
  bool debug_mode = false;
};

/// ⭐ 元素访问策略：**业务侧特化**，把 4 处 `Armor` 字段访问抽出来
/// @tparam T 检测元素类型（如 `auto_aim::Armor`）
template <typename T>
struct TemporalTraits;

/// 单个目标的时序积分结果（泛型 —— ⭐ E2 的 `Armor{0,0,...}` 初始化地雷消失）
template <typename T>
struct TIResult
{
  T item{};                                   ///< 最近一次的代表元素
  double confidence = 0.0;                     ///< 累积置信度 (0-1)
  int frame_count = 0;                         ///< 连续出现帧数
  bool verified = false;                       ///< 是否通过验证
  std::vector<cv::Point2f> history_positions;  ///< 图像坐标轨迹
};

/// @brief 多帧累积增强检测稳定性
template <typename T, typename Tr = TemporalTraits<T>>
class TemporalIntegrator
{
public:
  static constexpr size_t MAX_HISTORY_POS = 20;

  explicit TemporalIntegrator(const TIConfig & config = TIConfig()) : config_(config) {}

  /// @brief 处理当前帧
  /// @param items 当前帧检测结果
  /// @param timestamp 时间戳（当前未使用，保留接口）
  /// @param target_velocity 目标速度（m/s，<0 表示未知；用于自适应窗口）
  std::list<T> process(const std::list<T> & items, double timestamp, double target_velocity = -1.0)
  {
    (void)timestamp;
    ++frame_count_;

    // 1. 自适应窗口（⭐ B1 修复：velocity 一路传下去）
    const int current_window = adapt_window_size(target_velocity);
    if (current_window <= 0) {
      // 高速场景：旁路时序积分，直接返回原始结果
      if (config_.debug_mode && frame_count_ % 10 == 0)
        LOG_TI(
          "[TI] Frame {}: 高速 (v={:.1f} m/s) → 旁路 TI", frame_count_, target_velocity);
      return items;
    }

    // 2. 更新历史窗口
    update_history_window(items, current_window);

    // 3. 融合并输出「验证通过」的目标
    auto verified = fuse_history_window();

    // 4. 调试日志
    if (config_.debug_mode && frame_count_ % 10 == 0)
      LOG_TI(
        "[TI] Frame {}: window={} verified={}/{}", frame_count_, history_window_.size(),
        verified.size(), items.size());

    return verified;
  }

  /// @brief id -> 置信度
  std::unordered_map<int, double> getConfidences() const
  {
    std::unordered_map<int, double> out;
    for (const auto & [id, r] : tracked_targets_) out[id] = r.confidence;
    return out;
  }

  void reset()
  {
    history_window_.clear();
    tracked_targets_.clear();
    frame_count_ = verified_count_ = total_count_ = 0;
  }

  void setConfig(const TIConfig & config) { config_ = config; }

  std::string getStats() const
  {
    const double avg = total_count_ > 0 ? double(verified_count_) / total_count_ : 0.0;
    return fmt::format(
      "TI Stats: frames={}, window={}, verified={}/{}, avg_conf={:.2f}", frame_count_,
      history_window_.size(), verified_count_, total_count_, avg);
  }

  const TIConfig & config() const { return config_; }
  size_t window() const { return history_window_.size(); }

private:
  /// 自适应窗口大小（原 `adaptWindowSize`）
  int adapt_window_size(double velocity) const
  {
    if (!config_.adaptive_mode) return config_.window_size;
    if (velocity < 0) return config_.window_size;           // 速度未知 → 默认
    if (velocity < config_.low_speed_threshold) return config_.low_speed_window;
    if (velocity < config_.high_speed_threshold) return config_.mid_speed_window;
    return config_.enable_high_speed ? 2 : 0;               // 高速 → 默认旁路
  }

  /// 在历史窗口第 window_idx 帧里找空间最近的同类目标
  const T * find_match_in_history(const T & cur, int window_idx) const
  {
    if (window_idx < 0 || window_idx >= static_cast<int>(history_window_.size())) return nullptr;
    const T * best = nullptr;
    double min_dist = config_.spatial_tolerance;   // 只在容差内匹配
    for (const auto & it : history_window_[window_idx]) {
      if (!Tr::same_object(it, cur)) continue;     // ⭐ 原版注释强调：必须查 name，防跳变
      const double d = Tr::distance(cur, it);
      if (d < min_dist) {
        min_dist = d;
        best = &it;
      }
    }
    return best;
  }

  /// 置信度 = 在历史各帧中被匹配到的频率
  double compute_confidence(const T & item) const
  {
    const int total_frames =
      std::min(static_cast<int>(history_window_.size()), adapt_window_size(-1.0));
    // ⭐ B2 修复：window_size = 0 时 total_frames 会是 0 → 原版会除零得 NaN
    if (total_frames <= 0) return 0.0;

    int match_count = 0;
    for (int i = 0; i < total_frames; ++i)
      if (find_match_in_history(item, i) != nullptr) ++match_count;

    return static_cast<double>(match_count) / total_frames;
  }

  /// ⭐ B1 修复：窗口大小由调用方（已按 velocity 自适应）传入
  void update_history_window(const std::list<T> & items, int max_size)
  {
    history_window_.push_front(items);
    while (static_cast<int>(history_window_.size()) > max_size) history_window_.pop_back();
  }

  std::list<T> fuse_history_window()
  {
    std::list<T> fused;
    if (history_window_.empty()) return fused;

    for (const auto & item : history_window_.front()) {
      const double confidence = compute_confidence(item);
      const int id = Tr::id(item);

      auto & r = tracked_targets_[id];
      r.item = item;
      r.confidence = confidence;
      ++r.frame_count;
      r.history_positions.push_back(Tr::image_center(item));
      if (r.history_positions.size() > MAX_HISTORY_POS)
        r.history_positions.erase(r.history_positions.begin());

      r.verified = (confidence >= config_.confidence_threshold);
      if (r.verified) {
        ++verified_count_;
        Tr::set_confidence(const_cast<T &>(item), confidence);   // 回写融合置信度（可选）
        fused.push_back(item);
        if (config_.debug_mode)
          tools::logger()->debug(
            "[TI] 验证通过: ID={} conf={:.2f} frames={}", id, confidence, r.frame_count);
      }
      ++total_count_;
    }
    return fused;
  }

  TIConfig config_;
  std::deque<std::list<T>> history_window_;
  std::unordered_map<int, TIResult<T>> tracked_targets_;
  int frame_count_ = 0, verified_count_ = 0, total_count_ = 0;
};

}  // namespace tools

#endif  // HZMIR_UTILS_WHEELS_TRACK_TEMPORAL_INTEGRATOR_HPP
