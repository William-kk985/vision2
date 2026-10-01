/**
 * @file utils/wheels/estimate/angular_velocity.hpp
 * @brief ⭐ 轮子：**目标角速度 ω 的 4 种估计方法**（零业务依赖）
 *
 * ## 来源与演化
 * 原为 `Target` 的**策略槽位**（`IAngularVelocityEstimator` 虚接口 + 4 实现，W23 迁入）。
 * ⭐ **W43 按决定改为独立轮子** —— 去掉虚接口：
 * ```
 *   原：Target 里持 unique_ptr<IAngularVelocityEstimator> → 拷贝要 clone() → +40 ns/次 + 550 行
 *   现：**纯类**，谁需要谁直接实例化 → Target 回到同济原样（无槽位、无额外拷贝）
 * ```
 *
 * ## 为什么做成"轮子"而不是"槽位"
 * `Target` 的拷贝在**每帧热路径**上（`Planner::plan(Target)` 按值传）；
 * 而**调试**已由「宏 + `test/`」覆盖、**换算法**可以由「改一行 + 重编译」覆盖 ——
 * ⭐ 用不上的灵活性不值得放进热路径。
 *
 * ## 用法（谁需要谁调，无虚函数开销）
 * ```cpp
 * utils::wheels::VisualDiff w;
 * w.update(yaw, t);
 * double omega = w.estimate(ekf_x[7]);      // 数据不足时回退 EKF 的 ω
 * ```
 *
 * ⚠️ **默认 = 同济行为**：`Target` 直接用 EKF 状态的 `x[7]`（等价 `EkfStateOnly`）。
 */
#ifndef HZMIR_UTILS_WHEELS_ESTIMATE_ANGULAR_VELOCITY_HPP
#define HZMIR_UTILS_WHEELS_ESTIMATE_ANGULAR_VELOCITY_HPP

#include <algorithm>
#include <chrono>
#include <cmath>
#include <deque>

namespace utils::wheels
{

/// 差分类估计的公共历史（yaw + 时间戳）
class YawHistory
{
public:
  static constexpr std::size_t kWindow = 5;

  void push(double yaw, std::chrono::steady_clock::time_point t)
  {
    yaw_.push_back(yaw);
    t_.push_back(t);
    while (yaw_.size() > kWindow) {
      yaw_.pop_front();
      t_.pop_front();
    }
  }
  void clear()
  {
    yaw_.clear();
    t_.clear();
  }
  std::size_t size() const { return yaw_.size(); }
  bool ready(std::size_t n) const { return yaw_.size() >= n; }

  const std::deque<double> & yaw() const { return yaw_; }
  const std::deque<std::chrono::steady_clock::time_point> & t() const { return t_; }

private:
  std::deque<double> yaw_;
  std::deque<std::chrono::steady_clock::time_point> t_;
};

/// ⭐ 0. 同济行为：**直接用 EKF 状态的 ω**（不差分）
struct EkfStateOnly
{
  /// @param ekf_w EKF 状态里的角速度 `x[7]`
  double estimate(double ekf_w) const { return ekf_w; }
  void update(double, std::chrono::steady_clock::time_point) {}
};

/// ⭐ 1. 首尾两帧**中心差分** + 限幅（原 `Target::estimateInstantaneousAngularVelocity`）
class EkfOnly
{
public:
  static constexpr double kLimit = 10.0;

  void update(double yaw, std::chrono::steady_clock::time_point t) { h_.push(yaw, t); }

  /// @param ekf_w 数据不足时回退到它
  double estimate(double ekf_w) const
  {
    if (!h_.ready(3)) return ekf_w;
    return clamp(center_diff(h_.yaw().front(), h_.yaw().back(), h_.t().front(), h_.t().back()));
  }

private:
  static double clamp(double w) { return std::max(-kLimit, std::min(kLimit, w)); }
  static double center_diff(
    double y0, double y1, std::chrono::steady_clock::time_point t0,
    std::chrono::steady_clock::time_point t1)
  {
    const double dt = std::chrono::duration<double>(t1 - t0).count();
    if (dt <= 1e-6) return 0.0;
    double d = y1 - y0;
    while (d > M_PI) d -= 2 * M_PI;
    while (d < -M_PI) d += 2 * M_PI;
    return d / dt;
  }
  YawHistory h_;
};

/// ⭐ 2. 窗口内**所有相邻帧差分**，越近权重越高
class VisualDiff
{
public:
  static constexpr double kLimit = 10.0;

  void update(double yaw, std::chrono::steady_clock::time_point t) { h_.push(yaw, t); }

  double estimate(double ekf_w) const
  {
    if (!h_.ready(2)) return ekf_w;
    const auto & y = h_.yaw();
    const auto & ts = h_.t();
    double num = 0, den = 0;
    for (std::size_t i = 1; i < y.size(); ++i) {
      const double dt = std::chrono::duration<double>(ts[i] - ts[i - 1]).count();
      if (dt <= 1e-6) continue;
      double d = y[i] - y[i - 1];
      while (d > M_PI) d -= 2 * M_PI;
      while (d < -M_PI) d += 2 * M_PI;
      const double w = static_cast<double>(i);   // 越近权重越高
      num += w * (d / dt);
      den += w;
    }
    if (den <= 0) return ekf_w;
    return std::max(-kLimit, std::min(kLimit, num / den));
  }

private:
  YawHistory h_;
};

/// ⭐ 3. 互补融合：`alpha * 短期差分 + (1-alpha) * 长期差分`
class ImuFusion
{
public:
  static constexpr double kLimit = 10.0;

  explicit ImuFusion(double alpha = 0.7) : alpha_(alpha) {}

  void update(double yaw, std::chrono::steady_clock::time_point t) { h_.push(yaw, t); }

  double estimate(double ekf_w) const
  {
    if (!h_.ready(2)) return ekf_w;
    const auto & y = h_.yaw();
    const auto & ts = h_.t();
    const std::size_t n = y.size();
    const double short_w = diff(y[n - 2], y[n - 1], ts[n - 2], ts[n - 1]);
    const double long_w = diff(y.front(), y.back(), ts.front(), ts.back());
    const double fused = alpha_ * short_w + (1.0 - alpha_) * long_w;
    return std::max(-kLimit, std::min(kLimit, fused));
  }

private:
  static double diff(
    double y0, double y1, std::chrono::steady_clock::time_point t0,
    std::chrono::steady_clock::time_point t1)
  {
    const double dt = std::chrono::duration<double>(t1 - t0).count();
    if (dt <= 1e-6) return 0.0;
    double d = y1 - y0;
    while (d > M_PI) d -= 2 * M_PI;
    while (d < -M_PI) d += 2 * M_PI;
    return d / dt;
  }
  double alpha_;
  YawHistory h_;
};

}  // namespace utils::wheels

#endif  // HZMIR_UTILS_WHEELS_ESTIMATE_ANGULAR_VELOCITY_HPP
