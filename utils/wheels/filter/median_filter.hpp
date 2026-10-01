/**
 * @file utils/wheels/filter/median_filter.hpp
 * @brief ⭐ 轮子：**三维量测的中值滤波**（零业务依赖）
 *
 * ## 来源
 * 原为 `Target` 的策略槽位（`IMeasurementFilter` 的 `MedianFilter`，W23 迁入）。
 * ⭐ **W43 改为独立轮子** —— `Target` 恢复同济原样（**同济完全没有这一层**）。
 *
 * ## 说明
 * 逻辑与原 `MedianFilter` 一致：
 * 1. 把新量测追加进历史
 * 2. 窗口 `<= WINDOW_SIZE`
 * 3. **不足 3 帧时直接返回当前量测**（不做滤波）
 * 4. 对 x/y/z 三维**分别排序取中值**
 *
 * ⚠️ 这个槽位是 3 个里**最弱的一个**（20 行逻辑），
 *    若将来要用，**一个开关 + 一个窗口参数就够了**，不必再做成插件。
 */
#ifndef HZMIR_UTILS_WHEELS_FILTER_MEDIAN_FILTER_HPP
#define HZMIR_UTILS_WHEELS_FILTER_MEDIAN_FILTER_HPP

#include <algorithm>
#include <deque>
#include <vector>

#include <Eigen/Dense>

namespace utils::wheels
{

/// ⭐ 直通（= 同济行为）：不做任何滤波
struct PassthroughFilter
{
  Eigen::Vector3d filter(const Eigen::Vector3d & m) const { return m; }
  void clear() {}
};

/// ⭐ 三维中值滤波（抗检测抖动的离群值）
class MedianFilter
{
public:
  static constexpr std::size_t kWINDOW = 5;

  Eigen::Vector3d filter(const Eigen::Vector3d & m)
  {
    history_.push_back(m);
    while (history_.size() > kWINDOW) history_.pop_front();
    if (history_.size() < 3) return m;   // 数据不足 → 原样返回

    std::vector<double> x, y, z;
    x.reserve(history_.size());
    y.reserve(history_.size());
    z.reserve(history_.size());
    for (const auto & p : history_) {
      x.push_back(p[0]);
      y.push_back(p[1]);
      z.push_back(p[2]);
    }
    auto mid = [](std::vector<double> & v) {
      std::sort(v.begin(), v.end());
      return v[v.size() / 2];
    };
    return Eigen::Vector3d(mid(x), mid(y), mid(z));
  }

  void clear() { history_.clear(); }
  std::size_t size() const { return history_.size(); }

private:
  std::deque<Eigen::Vector3d> history_;
};

}  // namespace utils::wheels

#endif  // HZMIR_UTILS_WHEELS_FILTER_MEDIAN_FILTER_HPP
