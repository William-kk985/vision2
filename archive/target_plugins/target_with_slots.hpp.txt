#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <memory>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "core/types.hpp"
#include "core/auto_aim/target/target_plugins.hpp"   // ⭐ W23：3 个策略槽位
#include "utils/ekf/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  // ⭐⭐ E2 家族修复（W36）：这 5 个原来是**没有默认值**的 POD 成员，
  //   而 `Target() = default;` 与 `Target(x, vyaw, r, h)` **都不会初始化它们**
  //   → 读到的是**未初始化的内存**。
  //   实测踩到：`test_nav_bridge` 里 `Target t(4.0, 1.0, 0.2, 0.1);` 之后
  //   `t.name` 是垃圾值（`-1692937928`），**而且断言两边都用它会恒真** → 假通过。
  ArmorName name = ArmorName::not_armor;
  ArmorType armor_type = ArmorType::big;
  ArmorPriority priority = ArmorPriority::fifth;
  bool jumped = false;
  int last_id = 0;  // debug only

  Target() = default;

  // ⭐⭐ W23：加了 `unique_ptr` 槽位后，隐式拷贝构造被**删除** → `Target` 会变 move-only，
  //    而 `Planner::plan(Target target, ...)` 是**按值传** → 会编译不过。
  //    所以必须显式实现「深拷贝（clone 插件）」。
  //    ⚠️ 代价：每次拷贝要 clone 3 个插件（见 test_target_plugins 的实测）。
  Target(const Target & other);
  Target & operator=(const Target & other);
  Target(Target &&) noexcept = default;
  Target & operator=(Target &&) noexcept = default;

  // ⭐ W23：策略槽位注入（不设则用默认：EkfOnly / FixedNoise / PassthroughFilter）
  void set_angular_velocity_estimator(std::unique_ptr<IAngularVelocityEstimator> e);
  void set_process_noise_adapter(std::unique_ptr<IProcessNoiseAdapter> a);
  void set_measurement_filter(std::unique_ptr<IMeasurementFilter> f);
  const IAngularVelocityEstimator * angular_velocity_estimator() const
  {
    return angular_velocity_estimator_.get();
  }
  const IProcessNoiseAdapter * process_noise_adapter() const
  {
    return process_noise_adapter_.get();
  }
  const IMeasurementFilter * measurement_filter() const { return measurement_filter_.get(); }

  /// ⭐ W23：策略槽位估计出的角速度（供 tracker/可视化消费）
  double estimated_angular_velocity() const
  {
    return angular_velocity_estimator_ ? const_cast<IAngularVelocityEstimator *>(
                                           angular_velocity_estimator_.get())
                                           ->estimate()
                                       : 0.0;
  }
  /// ⭐ W23：上次 `predict()` 实际用的过程噪声（供 Debug 数据面）
  double last_q_v1() const { return last_q_v1_; }
  double last_q_v2() const { return last_q_v2_; }
  Target(
    const Armor & armor, std::chrono::steady_clock::time_point t, double radius, int armor_num,
    Eigen::VectorXd P0_dig);
  Target(double x, double vyaw, double radius, double h);

  void predict(std::chrono::steady_clock::time_point t);
  void predict(double dt);
  void update(const Armor & armor);

  Eigen::VectorXd ekf_x() const;
  const tools::ExtendedKalmanFilter & ekf() const;
  std::vector<Eigen::Vector4d> armor_xyza_list() const;

  bool diverged() const;

  bool convergened();

  bool isinit = false;

  bool checkinit();

private:
  // ⭐⭐ W36：同样补默认值 —— `Target(x, vyaw, r, h)` 只设了 `armor_num_`，
  //   其余这些**全是未初始化**的（`checkinit()` 会读 `t_`/`update_count_`）。
  int armor_num_ = 0;
  int switch_count_ = 0;
  int update_count_ = 0;

  bool is_switch_ = false, is_converged_ = false;

  tools::ExtendedKalmanFilter ekf_;

  // ⭐ W23：3 个策略槽位（可插拔；默认实现见 target_plugins.hpp）
  std::unique_ptr<IAngularVelocityEstimator> angular_velocity_estimator_;
  std::unique_ptr<IProcessNoiseAdapter> process_noise_adapter_;
  std::unique_ptr<IMeasurementFilter> measurement_filter_;
  double last_q_v1_ = 0.0, last_q_v2_ = 0.0;   // ⭐ W23：Debug 用
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP