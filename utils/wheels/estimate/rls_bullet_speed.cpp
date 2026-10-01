#include "utils/wheels/estimate/rls_bullet_speed.hpp"

#include <algorithm>
#include <cmath>
#include <numeric>
#include <vector>

#include "utils/log/logger.hpp"

namespace tools
{

RlsBulletSpeed::RlsBulletSpeed(const RlsBulletSpeedConfig & cfg) : cfg_(cfg)
{
  init();
  tools::logger()->info(
    "[RlsBulletSpeed] init: v0={:.2f} m/s, k_pre={:.3f}, bias_pre={:.2f}, P0={:.0f}",
    cfg_.nominal_speed, cfg_.k_pre, cfg_.bias_pre, cfg_.p0);
}

RlsBulletSpeed::RlsBulletSpeed(double nominal_speed) : RlsBulletSpeed([&] {
  RlsBulletSpeedConfig c;
  c.nominal_speed = nominal_speed;
  return c;
}())
{
}

/// ⭐ B4 修复：初始化路径**只有这一条**（原来 ctor 与 reset 各写一份且数值不一致）
void RlsBulletSpeed::init()
{
  k_ = cfg_.k_pre;
  bias_ = cfg_.bias_pre;
  theta_ << cfg_.k_pre, cfg_.bias_pre;
  P_ = Eigen::Matrix2d::Identity() * cfg_.p0;
  update_count_ = 0;
  consecutive_rejects_ = 0;
  history_.clear();
  // ⭐ B9 修复：reset 必须把时间基准也清掉，否则 reset 后第一批观测会因「时间戳倒退」被拒
  last_timestamp_ = 0;
  t_origin_ = 0;
  has_origin_ = false;
}

void RlsBulletSpeed::reset()
{
  init();
  tools::logger()->info("[RlsBulletSpeed] reset");
}

/// ⭐ B5 修复：λ 的自适应规则只在这里写一次
double RlsBulletSpeed::lambda_for(int n) const
{
  if (n <= cfg_.fast_until) return cfg_.lambda_fast;   // 极速学习
  if (n <= cfg_.mid_until) return cfg_.lambda_mid;     // 快速学习
  return cfg_.lambda_slow;                             // 稳定保持
}

bool RlsBulletSpeed::update(double distance, double fly_time, double timestamp)
{
  if (fly_time <= 1e-9 || distance <= 0) return false;

  if (timestamp < last_timestamp_) {
    tools::logger()->warn("[RlsBulletSpeed] 时间戳倒退，忽略本次观测");
    return false;
  }
  last_timestamp_ = timestamp;

  // ⭐ B8 修复：**只在首个样本时定原点**，之后不再改变（原来是滑动窗口最老样本 → 漂移）
  if (!has_origin_) {
    t_origin_ = timestamp;
    has_origin_ = true;
  }

  if (is_outlier(distance, fly_time)) {
    ++consecutive_rejects_;
    tools::logger()->debug(
      "[RlsBulletSpeed] 异常值剔除({}/{}): d={:.2f} m, t={:.3f} s", consecutive_rejects_,
      cfg_.max_consecutive_rejects, distance, fly_time);
    // ⭐ B10 修复：**打破自我锁死** —— 连续拒绝说明「统计基准本身已经不对了」，
    //   清空历史重新起步（否则 history 不更新 → σ 不变 → 永远拒绝）
    if (consecutive_rejects_ >= cfg_.max_consecutive_rejects) {
      tools::logger()->warn(
        "[RlsBulletSpeed] 连续拒绝 {} 次 → 清空历史重新起步（打破锁死）", consecutive_rejects_);
      history_.clear();
      consecutive_rejects_ = 0;
    }
    return false;
  }
  consecutive_rejects_ = 0;

  const double v_actual = distance / fly_time;
  if (v_actual < cfg_.speed_min || v_actual > cfg_.speed_max) {
    tools::logger()->warn(
      "[RlsBulletSpeed] 速度不合理: {:.2f} m/s (d={:.2f} m, t={:.3f} s)", v_actual, distance,
      fly_time);
    return false;
  }

  // 回归：v = v0 − k·elapsed − bias  →  y = v0 − v_actual = k·elapsed + bias
  const double elapsed = timestamp - t_origin_;   // ⭐ B8：固定原点
  const Eigen::Vector2d phi(elapsed, 1.0);              // ⭐ B6：定长向量，无堆分配
  const double y = cfg_.nominal_speed - v_actual;

  theta_ = rls_update(phi, y);
  k_ = std::clamp(theta_[0], cfg_.k_min, cfg_.k_max);
  bias_ = std::clamp(theta_[1], cfg_.bias_min, cfg_.bias_max);

  history_.emplace_back(distance, fly_time, timestamp);
  if (history_.size() > cfg_.max_history) history_.pop_front();
  ++update_count_;

  if (update_count_ % 10 == 0 || update_count_ <= 3)
    tools::logger()->info(
      "[RlsBulletSpeed] k={:.4f} bias={:.2f} v_now={:.2f} m/s (n={}, λ={:.2f})", k_, bias_,
      current_speed(), update_count_, lambda_for(update_count_));
  return true;
}

Eigen::Vector2d RlsBulletSpeed::rls_update(const Eigen::Vector2d & phi, double y)
{
  const double lambda = lambda_for(update_count_ + 1);

  // K = P·φ / (λ + φᵀ·P·φ)
  const Eigen::Vector2d P_phi = P_ * phi;
  const double denom = lambda + phi.dot(P_phi);
  const Eigen::Vector2d K = P_phi / denom;

  // θ ← θ + K·(y − φᵀ·θ)
  theta_ += K * (y - phi.dot(theta_));

  // P ← (P − K·φᵀ·P) / λ，并强制对称（数值稳定）
  P_ = (P_ - K * phi.transpose() * P_) / lambda;
  P_ = (P_ + P_.transpose()) / 2.0;
  return theta_;
}

double RlsBulletSpeed::current_speed() const
{
  if (!has_origin_) return cfg_.nominal_speed;
  const double elapsed = last_timestamp_ - t_origin_;   // ⭐ B8：与回归同一原点
  return cfg_.nominal_speed - k_ * elapsed - bias_;
}

bool RlsBulletSpeed::is_outlier(double distance, double fly_time) const
{
  // ⭐ B10 修复①：样本太少时**不做**检测（原来固定 3 个样本就开判，σ 被严重低估）
  if (history_.size() < cfg_.outlier_min_samples) return false;

  std::vector<double> speeds;
  speeds.reserve(history_.size());
  for (const auto & [d, t, ts] : history_) speeds.push_back(d / t);

  const double mean = std::accumulate(speeds.begin(), speeds.end(), 0.0) / speeds.size();
  double var = 0.0;
  for (double s : speeds) var += (s - mean) * (s - mean);
  var /= speeds.size();
  // ⭐ B10 修复②：σ 加下限 —— 物理上弹速观测不可能比 `outlier_sigma_floor` 更稳
  const double sd = std::max(std::sqrt(var), cfg_.outlier_sigma_floor);

  return std::abs(distance / fly_time - mean) > cfg_.outlier_sigma * sd;
}

}  // namespace tools
