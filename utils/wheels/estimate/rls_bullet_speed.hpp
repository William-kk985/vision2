/**
 * @file utils/wheels/estimate/rls_bullet_speed.hpp
 * @brief ⭐ 轮子：RLS 弹速在线辨识（**你的算法**）
 *
 * **来源**：`Hz_Rm_Vision/src/core/aiming/bullet_speed_estimator.{hpp,cpp}`（108 + 180 行）
 * **落位理由**：`grep` 证明它**零业务类型依赖**（只需 `<Eigen>` / `<deque>` / `<tuple>`）
 *   —— 原文件放在 `namespace auto_aim` 里，但它其实一个 `auto_aim::` 类型都没用。
 *
 * ## 模型
 * ```
 *   v(t) = v0 − k·t − bias
 *   φ = [elapsed_time, 1]，y = v0 − v_actual = v0 − d/t
 *   θ = [k, bias]  ← RLS 递推（λ 自适应遗忘因子）
 * ```
 *
 * ## ⭐ 移植时修的 4 处（都是原实现里能看出问题的地方）
 *
 * | # | 原问题 | 本实现 |
 * |---|---|---|
 * | **B4** | ⭐ **`ctor` 与 `reset()` 不一致**：构造用 `P=I·200, θ=[0.08,0.3]`，<br>`reset()` 却用 `P=I·1000, θ=0` → **重置后估计器与刚启动时不是同一个东西** | 两者都走同一个 `init()`，参数进 `Config` |
 * | **B5** | ⭐ 自适应 λ 的**同一段逻辑写了两遍**（`rls_update` 里一份、`update` 里再算一份仅用于打日志） | 抽成 `lambda_for(n)` |
 * | **B6** | `Eigen::VectorXd phi(2)` —— 定长向量用动态矩阵（堆分配） | `Eigen::Vector2d` |
 * | **B7** | 参数全部**硬编码**在 .cpp 里（`200/0.85/0.92/0.98/10/40/±5/3σ`） | 全部进 `RlsBulletSpeedConfig`（可从 yaml 配） |
 * | **B8** | ⭐⭐ **回归原点会漂**：`elapsed = ts − history_.front().ts`，而 `history_` 是**滑动窗口** →<br>原点一直变 → `y = k·elapsed + bias` **不可辨识**，`k`/`bias` 有偏（实测 k=0.597 vs 真值 0.5） | 用**固定原点** `t_origin_`（首个样本时刻，直到 reset） |
 * | **B9** | ⭐ `init()` 没清 `last_timestamp_` → **reset 后第一批观测会因"时间戳倒退"被拒**<br>（实测 reset 后结果与 fresh 不一致） | `init()` 一并清原点与时间戳 |
 * | **B10** | ⭐⭐⭐ **异常值检测会自我锁死**：`is_outlier` 用**样本标准差**（3~4 个样本时严重低估噪声，<br>实测 σ≈0.11 而真实噪声 0.3）→ 正常观测被判异常 →<br>**而拒绝时 `history_` 不更新 → σ 永远不变 → 之后全部拒绝**（实测 60 次只吸收 4 次） | ① 样本数下限可配（默认 5）② **σ 下限**（默认 0.5 m/s）<br>③ ⭐ **连续拒绝计数**：超过阈值就清空历史重新起步（**打破锁死**） |
 *
 * ⚠️ 与同济的关系：同济**没有**弹速在线辨识（`Planner` 只从 yaml/下位机读 `bullet_speed`）。
 *    这是本项目的增量能力。
 */
#ifndef HZMIR_UTILS_WHEELS_ESTIMATE_RLS_BULLET_SPEED_HPP
#define HZMIR_UTILS_WHEELS_ESTIMATE_RLS_BULLET_SPEED_HPP

#include <deque>
#include <tuple>

#include <Eigen/Dense>

namespace tools
{

/// 全部可调参数（原来散落在 .cpp 里的魔法数字）
struct RlsBulletSpeedConfig
{
  double nominal_speed = 25.0;   ///< 标称初速 v0（m/s）
  int min_updates = 5;           ///< 收敛判据（更新次数）

  // ── ⭐ B4：初始化（ctor 与 reset 必须一致）──
  double p0 = 200.0;             ///< 初始协方差（原 ctor 用 200，reset 用 1000）
  double k_pre = 0.08;           ///< 衰减系数先验
  double bias_pre = 0.3;         ///< 偏差先验

  // ── 自适应遗忘因子 λ（越小学得越快）──
  double lambda_fast = 0.85;     ///< 前 `fast_until` 次
  double lambda_mid = 0.92;      ///< 到 `mid_until` 次
  double lambda_slow = 0.98;     ///< 之后
  int fast_until = 2;
  int mid_until = 5;

  // ── 物理约束 ──
  double speed_min = 10.0, speed_max = 40.0;   ///< 实际速度合理区间
  double k_min = 0.0, k_max = 10.0;            ///< 衰减系数钳位
  double bias_min = -5.0, bias_max = 5.0;      ///< 偏差钳位
  // ── 异常值剔除（⭐ B10：原实现会自我锁死）──
  double outlier_sigma = 3.0;      ///< 3σ 判据
  size_t outlier_min_samples = 5;  ///< ⭐ 样本少于此数不做检测（原来固定 3，σ 被严重低估）
  double outlier_sigma_floor = 0.5;///< ⭐ σ 下限（m/s）—— 弹速观测不可能比这更稳
  int max_consecutive_rejects = 5; ///< ⭐ 连续拒绝这么多次 → **清空历史打破锁死**

  size_t max_history = 20;         ///< 历史窗口
};

class RlsBulletSpeed
{
public:
  explicit RlsBulletSpeed(const RlsBulletSpeedConfig & cfg = RlsBulletSpeedConfig());

  /// 便捷构造：只给标称初速（保留原 `BulletSpeedEstimator(25.0)` 的用法）
  explicit RlsBulletSpeed(double nominal_speed);

  /// @brief 用一次「距离 + 飞行时间」观测更新估计
  /// @param distance 目标水平距离（m）
  /// @param fly_time 弹丸飞行时间（s）
  /// @param timestamp 时间戳（s，单调递增）
  /// @return 本次是否真的吸收了该观测（false = 被判为异常/不合理/时间倒流）
  bool update(double distance, double fly_time, double timestamp);

  /// @brief 当前估计速度 `v0 − k·elapsed − bias`
  double current_speed() const;

  double nominal_speed() const { return cfg_.nominal_speed; }
  double decay() const { return k_; }     ///< k
  double bias() const { return bias_; }
  bool converged() const { return update_count_ > cfg_.min_updates; }
  int update_count() const { return update_count_; }

  void reset();
  void set_nominal_speed(double v0) { cfg_.nominal_speed = v0; }

  const RlsBulletSpeedConfig & config() const { return cfg_; }
  void set_config(const RlsBulletSpeedConfig & cfg) { cfg_ = cfg; }

private:
  void init();                                   ///< ⭐ B4：ctor 与 reset 共用
  double lambda_for(int n) const;                ///< ⭐ B5：λ 只写一处
  Eigen::Vector2d rls_update(const Eigen::Vector2d & phi, double y);
  bool is_outlier(double distance, double fly_time) const;

  RlsBulletSpeedConfig cfg_;
  double k_ = 0.0, bias_ = 0.0;
  Eigen::Matrix2d P_ = Eigen::Matrix2d::Identity();
  Eigen::Vector2d theta_ = Eigen::Vector2d::Zero();
  int update_count_ = 0;
  int consecutive_rejects_ = 0;   // ⭐ B10：连续拒绝计数
  double last_timestamp_ = 0;
  // ⭐ B8：**固定回归原点**（首个样本时刻）。原实现用滑动窗口最老样本当原点 → 原点漂移 → 不可辨识
  double t_origin_ = 0;
  bool has_origin_ = false;
  std::deque<std::tuple<double, double, double>> history_;   // (d, t, ts)
};

}  // namespace tools

#endif  // HZMIR_UTILS_WHEELS_ESTIMATE_RLS_BULLET_SPEED_HPP
