/**
 * @file core/auto_aim/target/target_plugins.hpp
 * @brief ⭐⭐ 估计器的 3 个可插拔策略槽位（**你的设计贡献**）
 *
 * **来源**：`Hz_Rm_Vision/src/core/tracking/target_plugins.hpp`（459 行）
 * **落位**：`core/auto_aim/target/`（`Target` 的专属策略，不是通用工具 → 不进 `utils/`）
 *
 * ## 为什么需要它（对照同济 `Target`）
 *
 * | 槽位 | 同济现状 | 你的插件补上的 |
 * |---|---|---|
 * | `IAngularVelocityEstimator` | 直接用 EKF 状态 `x[7]`（w），**无历史差分** | 历史窗口中心差分 / 加权差分 / 短长期互补融合 |
 * | `IProcessNoiseAdapter` | ⭐ **`v1`/`v2` 是硬编码局部变量**（前哨站 10/0.1，其他 100/400） | 按 NIS 自适应调整 Q |
 * | `IMeasurementFilter` | **完全没有** | 三维中值滤波 |
 *
 * ## ⚠️ 两处「命名与行为不符」（保留原名，仅记录）
 * - `EkfOnly` —— 实际做的是**中心差分**，只在历史 < 3 帧时回退到 EKF 的 w 值
 * - `ImuFusion` —— 实际**不使用 IMU**，是「短期差分 + 长期差分」互补融合（注释自述"模拟 IMU 融合效果"）
 *
 * ## ⚠️ 代价：`clone()` 让 `Target` 拷贝变贵
 * 三个槽位都是 `unique_ptr` + 有状态 → `Target` 拷贝要 `clone()` 三次。
 * W11 实测：同济 `Target` 仅 304 B / 拷贝 318 ns；加插件后会上一个量级。
 * **建议：`Target` 传递一律用移动/引用，别按值传**（`Planner::plan(Target target, ...)` 就是按值传，是隐患）。
 */
#ifndef HZMIR_CORE_AUTO_AIM_TARGET_TARGET_PLUGINS_HPP
#define HZMIR_CORE_AUTO_AIM_TARGET_TARGET_PLUGINS_HPP


#include <algorithm>
#include <cmath>
#include <deque>
#include <chrono>
#include <memory>   // ⭐ W23：原文件靠传递包含侥幸编过（同 C23 一类）
#include <numeric>
#include <vector>
#include <Eigen/Dense>

#include "utils/math/chi2.hpp"   // ⭐ W24：阈值用 χ² 分位表达

namespace auto_aim {

// ============================================================
// 接口 1: 角速度估计策略
// Requirements: 4.1
// ============================================================
class IAngularVelocityEstimator {
public:
  virtual ~IAngularVelocityEstimator() = default;

  /**
   * @brief 添加新的 yaw 观测值（由 Target::update() 调用）
   * @param yaw        当前 yaw 角度
   * @param timestamp  对应时间戳
   * 
   * 历史数据由插件内部管理（Requirements: 2.1）
   */
  virtual void update(double yaw, std::chrono::steady_clock::time_point timestamp) = 0;

  /**
   * @brief 估计目标角速度（使用内部历史数据）
   * @return 估计的角速度（rad/s）
   */
  virtual double estimate() = 0;

  /**
   * @brief 获取 EKF 角速度状态（供 EkfOnly 使用）
   * 默认返回 0，子类可重写以注入 EKF 状态
   */
  virtual double ekf_angular_velocity() const { return 0.0; }

  /**
   * @brief 设置 EKF 角速度状态（供 EkfOnly 使用）
   */
  virtual void set_ekf_angular_velocity(double w) { (void)w; }

  /**
   * @brief 克隆当前策略对象（用于 Target 拷贝构造）
   */
  virtual std::unique_ptr<IAngularVelocityEstimator> clone() const = 0;
};

// ------------------------------------------------------------
// 实现 1.0: EkfStateOnly — ⭐ **同济行为的严格等价物**
//   同济 `Target` **不使用任何历史差分**，角速度直接取 EKF 状态 `x[7]`。
//   所以「默认」必须是这个，而不是 `EkfOnly`（后者会做首尾差分 → 行为已改变）。
// ------------------------------------------------------------
class EkfStateOnly : public IAngularVelocityEstimator
{
public:
  void update(double /*yaw*/, std::chrono::steady_clock::time_point /*t*/) override {}
  double estimate() override { return ekf_w_; }
  double ekf_angular_velocity() const override { return ekf_w_; }
  void set_ekf_angular_velocity(double w) override { ekf_w_ = w; }
  std::unique_ptr<IAngularVelocityEstimator> clone() const override
  {
    return std::make_unique<EkfStateOnly>(*this);
  }

private:
  double ekf_w_ = 0.0;
};

// ------------------------------------------------------------
// 实现 1.1: EkfOnly — 仅使用 EKF 状态（当前默认行为）
// Requirements: 4.4, 4.10
// 迁移自 Target::estimateInstantaneousAngularVelocity()
// 历史数据由插件内部管理（Requirements: 2.1）
// ------------------------------------------------------------
class EkfOnly : public IAngularVelocityEstimator {
public:
  static constexpr std::size_t HISTORY_SIZE = 5;  ///< 历史窗口大小

  EkfOnly() : ekf_w_(0.0) {}

  /**
   * @brief 添加新的 yaw 观测值到内部历史队列
   */
  void update(double yaw, std::chrono::steady_clock::time_point timestamp) override
  {
    yaw_history_.push_back(yaw);
    timestamp_history_.push_back(timestamp);

    if (yaw_history_.size() > HISTORY_SIZE) {
      yaw_history_.pop_front();
      timestamp_history_.pop_front();
    }
  }

  /**
   * @brief 使用内部历史数据估计角速度，数据不足时回退到 EKF 状态
   *
   * 逻辑与原 Target::estimateInstantaneousAngularVelocity() 完全一致：
   * - 历史数据不足 3 帧时返回 EKF 状态值
   * - 使用首尾两帧做中心差分，限幅到 [-10, 10] rad/s
   */
  double estimate() override
  {
    if (yaw_history_.size() < 3 || timestamp_history_.size() < 3) {
      return ekf_w_;  // 数据不足，返回 EKF 估计
    }

    double yaw_old = yaw_history_.front();
    double yaw_new = yaw_history_.back();

    auto dt_old = timestamp_history_.front();
    auto dt_new = timestamp_history_.back();
    double dt = std::chrono::duration<double>(dt_new - dt_old).count();

    // 保护：dt 过小会导致数值不稳定
    if (dt < 0.001) {
      return ekf_w_;
    }

    // 角度差归一化到 [-π, π]（等价于原代码 tools::limit_rad）
    double delta_yaw = yaw_new - yaw_old;
    while (delta_yaw >  M_PI) delta_yaw -= 2.0 * M_PI;
    while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;

    double instantaneous_w = delta_yaw / dt;

    // 限幅：物理上不可能超过 10 rad/s
    instantaneous_w = std::max(-10.0, std::min(10.0, instantaneous_w));

    return instantaneous_w;
  }

  double ekf_angular_velocity() const override { return ekf_w_; }
  void set_ekf_angular_velocity(double w) override { ekf_w_ = w; }

  std::unique_ptr<IAngularVelocityEstimator> clone() const override {
    return std::make_unique<EkfOnly>(*this);
  }

private:
  double ekf_w_;  ///< EKF 估计的角速度，由 Target 在每次 predict 后注入
  std::deque<double> yaw_history_;  ///< 内部历史 yaw 角度队列（Requirements: 2.1）
  std::deque<std::chrono::steady_clock::time_point> timestamp_history_;  ///< 内部时间戳队列（Requirements: 2.1）
};

// ------------------------------------------------------------
// 实现 1.2: VisualDiff — 纯视觉差分估计
// Requirements: 4.4
// 使用滑动窗口内所有相邻帧的加权差分均值，比 EkfOnly 更平滑
// 历史数据由插件内部管理（Requirements: 2.1）
// ------------------------------------------------------------
class VisualDiff : public IAngularVelocityEstimator {
public:
  static constexpr std::size_t HISTORY_SIZE = 5;  ///< 历史窗口大小

  /**
   * @brief 添加新的 yaw 观测值到内部历史队列
   */
  void update(double yaw, std::chrono::steady_clock::time_point timestamp) override
  {
    yaw_history_.push_back(yaw);
    timestamp_history_.push_back(timestamp);

    if (yaw_history_.size() > HISTORY_SIZE) {
      yaw_history_.pop_front();
      timestamp_history_.pop_front();
    }
  }

  /**
   * @brief 对窗口内所有相邻帧做差分，取加权平均（越近的帧权重越高）
   */
  double estimate() override
  {
    if (yaw_history_.size() < 2 || timestamp_history_.size() < 2) {
      return 0.0;
    }

    double weighted_sum = 0.0;
    double weight_total = 0.0;
    const int n = static_cast<int>(yaw_history_.size());

    for (int i = 1; i < n; ++i) {
      double dt = std::chrono::duration<double>(timestamp_history_[i] - timestamp_history_[i - 1]).count();
      if (dt < 1e-6) continue;

      double delta_yaw = yaw_history_[i] - yaw_history_[i - 1];
      while (delta_yaw >  M_PI) delta_yaw -= 2.0 * M_PI;
      while (delta_yaw < -M_PI) delta_yaw += 2.0 * M_PI;

      // 越近的帧权重越高（线性递增）
      double w_i = static_cast<double>(i);
      weighted_sum += w_i * (delta_yaw / dt);
      weight_total += w_i;
    }

    if (weight_total < 1e-9) return 0.0;

    double result = weighted_sum / weight_total;
    // 限幅
    return std::max(-10.0, std::min(10.0, result));
  }

  std::unique_ptr<IAngularVelocityEstimator> clone() const override {
    return std::make_unique<VisualDiff>(*this);
  }

private:
  std::deque<double> yaw_history_;  ///< 内部历史 yaw 角度队列（Requirements: 2.1）
  std::deque<std::chrono::steady_clock::time_point> timestamp_history_;  ///< 内部时间戳队列（Requirements: 2.1）
};

// ------------------------------------------------------------
// 实现 1.3: ImuFusion — 基于视觉历史的自适应融合估计
// Requirements: 4.4
// 在接口约束（仅 yaw_history + timestamps）内，通过对短期/长期
// 差分做互补融合，模拟 IMU 高频 + 视觉低频的融合效果：
//   - 短期差分（最近 2 帧）：响应快，噪声大，类比 IMU 高频分量
//   - 长期差分（首尾帧）：响应慢，噪声小，类比视觉低频分量
// 历史数据由插件内部管理（Requirements: 2.1）
// ------------------------------------------------------------
class ImuFusion : public IAngularVelocityEstimator {
public:
  static constexpr std::size_t HISTORY_SIZE = 5;  ///< 历史窗口大小

  /**
   * @param alpha  短期差分权重，默认 0.7（偏向快速响应）
   */
  explicit ImuFusion(double alpha = 0.7) : alpha_(alpha) {}

  /**
   * @brief 添加新的 yaw 观测值到内部历史队列
   */
  void update(double yaw, std::chrono::steady_clock::time_point timestamp) override
  {
    yaw_history_.push_back(yaw);
    timestamp_history_.push_back(timestamp);

    if (yaw_history_.size() > HISTORY_SIZE) {
      yaw_history_.pop_front();
      timestamp_history_.pop_front();
    }
  }

  /**
   * @brief 互补融合：alpha * 短期差分 + (1-alpha) * 长期差分
   */
  double estimate() override
  {
    if (yaw_history_.size() < 2 || timestamp_history_.size() < 2) {
      return 0.0;
    }

    // 短期差分：最近两帧（高频，低延迟）
    double short_term_w = 0.0;
    {
      const int n = static_cast<int>(yaw_history_.size());
      double dt = std::chrono::duration<double>(
        timestamp_history_[n - 1] - timestamp_history_[n - 2]).count();
      if (dt > 1e-6) {
        double delta = yaw_history_[n - 1] - yaw_history_[n - 2];
        while (delta >  M_PI) delta -= 2.0 * M_PI;
        while (delta < -M_PI) delta += 2.0 * M_PI;
        short_term_w = delta / dt;
      }
    }

    // 长期差分：首尾帧（低频，稳定）
    double long_term_w = short_term_w;  // 默认与短期相同
    if (yaw_history_.size() >= 3) {
      double dt = std::chrono::duration<double>(
        timestamp_history_.back() - timestamp_history_.front()).count();
      if (dt > 1e-6) {
        double delta = yaw_history_.back() - yaw_history_.front();
        while (delta >  M_PI) delta -= 2.0 * M_PI;
        while (delta < -M_PI) delta += 2.0 * M_PI;
        long_term_w = delta / dt;
      }
    }

    // 互补融合
    double fused = alpha_ * short_term_w + (1.0 - alpha_) * long_term_w;

    // 限幅
    return std::max(-10.0, std::min(10.0, fused));
  }

  std::unique_ptr<IAngularVelocityEstimator> clone() const override {
    return std::make_unique<ImuFusion>(*this);
  }

private:
  double alpha_;  ///< 短期差分权重
  std::deque<double> yaw_history_;  ///< 内部历史 yaw 角度队列（Requirements: 2.1）
  std::deque<std::chrono::steady_clock::time_point> timestamp_history_;  ///< 内部时间戳队列（Requirements: 2.1）
};

// ============================================================
// 接口 2: 过程噪声自适应策略
// Requirements: 4.2
// ============================================================
class IProcessNoiseAdapter {
public:
  virtual ~IProcessNoiseAdapter() = default;

  /**
   * @brief 根据 NIS 值自适应调整过程噪声参数
   * @param nis  归一化新息平方（Normalized Innovation Squared）
   * @param v1   过程噪声参数 1（可被修改）
   * @param v2   过程噪声参数 2（可被修改）
   */
  virtual void adapt(double nis, double& v1, double& v2) = 0;

  /**
   * @brief 克隆当前策略对象（用于 Target 拷贝构造）
   */
  virtual std::unique_ptr<IProcessNoiseAdapter> clone() const = 0;
};

// ------------------------------------------------------------
// 实现 2.1: FixedNoise — 固定噪声（当前默认行为）
// Requirements: 4.5
// adapt() 方法体为空，不修改 v1/v2
// ------------------------------------------------------------
class FixedNoise : public IProcessNoiseAdapter {
public:
  void adapt(double /*nis*/, double& /*v1*/, double& /*v2*/) override
  {
    // 固定噪声：不修改 v1, v2，保持传入值不变
  }

  std::unique_ptr<IProcessNoiseAdapter> clone() const override {
    return std::make_unique<FixedNoise>(*this);
  }
};

// ------------------------------------------------------------
// 实现 2.2: NisBasedAdapter — 基于 NIS 的自适应噪声
// Requirements: 4.5, 4.10
// 迁移自 Target::adapt_process_noise()
// ------------------------------------------------------------
class NisBasedAdapter : public IProcessNoiseAdapter {
public:
  /**
   * ⭐⭐ B13 修复说明（保留原类名与字段语义）
   *
   * 原实现在**普通目标**（默认 `v1=100, v2=400`）上方向是**反的**：
   * ```
   * NIS 略大（"应提高 Q"）： v2 = min(300, 400*1.1) = 300   ← 反而【降低】了！
   * NIS 很大（"应提高 Q"）： v2 = min(400, 400*1.2) = 400   ← 不变
   * ```
   * 因为上限 `300/400` 与默认值 `400` 冲突。**`v1` 不受影响**（上限 150 > 默认 100）。
   *
   * 修复：把上下限做成**可配**且默认高于基线（`v2_max = 800`），
   *       让「NIS 大 → 提高 Q」在普通目标上也真正生效。
   */
  /// ⚠️ 这里用**显式构造函数**而非 NSDMI ——
  ///    GCC 不允许「嵌套类的 NSDMI」出现在「外层类的默认实参」里
  ///    （报 "default member initializer ... required before the end of its enclosing class"）。
  struct Limits
  {
    double v1_min, v1_max;
    double v2_min, v2_max;   // ⭐ B13：原为 150/300/400（上限与默认 400 冲突）
    Limits(double a = 5.0, double b = 150.0, double c = 150.0, double d = 800.0)
    : v1_min(a), v1_max(b), v2_min(c), v2_max(d)
    {
    }
  };

  // ⭐ W24：阈值来自 χ²(4) 的 95% 分位（= 9.4877），不再是魔法数字 9.21
  //    （W9 实测：9.21 已是三处尺度中最接近正确值的，这里完成最后一步收敛）
  explicit NisBasedAdapter(
    Limits lim = Limits(), double nis_threshold = tools::chi2_q95(4))
  : lim_(lim), nis_threshold_(nis_threshold)
  {
  }
  /**
   * @brief 根据 NIS 值动态调整过程噪声 v1（位置）和 v2（角速度）
   *
   * 逻辑与原 Target::adapt_process_noise() 完全一致：
   * - NIS > 1.5 * threshold：温和提高 Q（每帧 +20%）
   * - NIS > threshold：小幅增加（每帧 +10%）
   * - NIS < 0.5 * threshold：快速恢复（每帧 -20%）
   * - NIS 在 [0.5, 1.5]×χ²(4) 的 95% 分位之间时保持不变（即 [4.744, 14.232]）
   */
  void adapt(double nis, double& v1, double& v2) override
  {
    if (nis > nis_threshold_ * 1.5) {
      // NIS 严重过大：温和提高 Q
      v1 = std::min(lim_.v1_max, v1 * 1.2);
      v2 = std::min(lim_.v2_max, v2 * 1.2);
    } else if (nis > nis_threshold_) {
      // NIS 略大：小幅增加
      v1 = std::min(lim_.v1_max, v1 * 1.1);
      v2 = std::min(lim_.v2_max, v2 * 1.1);
    } else if (nis < nis_threshold_ * 0.5) {
      // NIS 很小：快速恢复
      v1 = std::max(lim_.v1_min, v1 * 0.8);
      v2 = std::max(lim_.v2_min, v2 * 0.8);
    }
    // NIS 在 [0.5·阈值, 阈值] 之间时保持不变（死区）
  }

  const Limits & limits() const { return lim_; }
  double nis_threshold() const { return nis_threshold_; }

private:
  Limits lim_;
  double nis_threshold_ = tools::chi2_q95(4);   // ⭐ W24：9.4877（原 9.21）
public:

  std::unique_ptr<IProcessNoiseAdapter> clone() const override {
    return std::make_unique<NisBasedAdapter>(*this);
  }
};

// ============================================================
// 接口 3: 观测值预处理策略
// Requirements: 4.3
// ============================================================
class IMeasurementFilter {
public:
  virtual ~IMeasurementFilter() = default;

  /**
   * @brief 对观测值进行预处理/滤波
   * @param measurement  当前观测值（3D 位置）
   * @return 经过滤波处理后的观测值
   * 
   * 历史数据由插件内部管理（Requirements: 2.1）
   */
  virtual Eigen::Vector3d filter(const Eigen::Vector3d& measurement) = 0;

  /**
   * @brief 克隆当前策略对象（用于 Target 拷贝构造）
   */
  virtual std::unique_ptr<IMeasurementFilter> clone() const = 0;
};

// ------------------------------------------------------------
// 实现 3.1: PassthroughFilter — 直通（不过滤）
// Requirements: 4.6
// 直接返回 measurement
// ------------------------------------------------------------
class PassthroughFilter : public IMeasurementFilter {
public:
  Eigen::Vector3d filter(const Eigen::Vector3d& measurement) override
  {
    return measurement;
  }

  std::unique_ptr<IMeasurementFilter> clone() const override {
    return std::make_unique<PassthroughFilter>(*this);
  }
};

// ------------------------------------------------------------
// 实现 3.2: MedianFilter — 中值滤波（当前默认行为）
// Requirements: 4.6, 4.10
// 迁移自 Target::medianFilterPosition()
// 历史数据由插件内部管理（Requirements: 2.1）
// ------------------------------------------------------------
class MedianFilter : public IMeasurementFilter {
public:
  static constexpr std::size_t WINDOW_SIZE = 5;  ///< 与原 MEDIAN_FILTER_WINDOW 一致

  /**
   * @brief 对 x/y/z 三个维度分别进行中值滤波
   *
   * 逻辑与原 Target::medianFilterPosition() 完全一致：
   * 1. 将新测量值追加到内部历史队列
   * 2. 保持窗口大小 <= WINDOW_SIZE
   * 3. 数据不足 3 帧时直接返回当前测量值
   * 4. 对三个维度分别排序取中值
   */
  Eigen::Vector3d filter(const Eigen::Vector3d& measurement) override
  {
    history_.push_back(measurement);
    if (history_.size() > WINDOW_SIZE) {
      history_.pop_front();
    }

    if (history_.size() < 3) {
      return measurement;
    }

    std::vector<double> x_vals, y_vals, z_vals;
    x_vals.reserve(history_.size());
    y_vals.reserve(history_.size());
    z_vals.reserve(history_.size());

    for (const auto& pos : history_) {
      x_vals.push_back(pos.x());
      y_vals.push_back(pos.y());
      z_vals.push_back(pos.z());
    }

    std::sort(x_vals.begin(), x_vals.end());
    std::sort(y_vals.begin(), y_vals.end());
    std::sort(z_vals.begin(), z_vals.end());

    std::size_t mid = history_.size() / 2;
    return Eigen::Vector3d(x_vals[mid], y_vals[mid], z_vals[mid]);
  }

  std::unique_ptr<IMeasurementFilter> clone() const override {
    return std::make_unique<MedianFilter>(*this);
  }

private:
  std::deque<Eigen::Vector3d> history_;  ///< 内部历史观测值队列
};

} // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_TARGET_TARGET_PLUGINS_HPP
