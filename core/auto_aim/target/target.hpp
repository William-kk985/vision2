#ifndef AUTO_AIM__TARGET_HPP
#define AUTO_AIM__TARGET_HPP

#include <Eigen/Dense>
#include <chrono>
#include <optional>
#include <queue>
#include <string>
#include <vector>

#include "core/types.hpp"
#include "utils/ekf/extended_kalman_filter.hpp"

namespace auto_aim
{

class Target
{
public:
  // ⭐⭐ E2 家族修复（保留）：这 5 个原来**没有默认值**，而 `Target() = default;`
  //   与 `Target(x, vyaw, r, h)` **都不初始化它们** → 读到未初始化内存。
  //   （实测踩到：`t.name` 是垃圾值，且断言两边都用它会恒真 → 假通过）
  ArmorName name = ArmorName::not_armor;
  ArmorType armor_type = ArmorType::big;
  ArmorPriority priority = ArmorPriority::fifth;
  bool jumped = false;
  int last_id = 0;  // debug only

  Target() = default;
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
  // ⭐⭐ E2 家族修复（保留）：`Target(x, vyaw, r, h)` 只设 `armor_num_`，其余原来全未初始化
  int armor_num_ = 0;

  // ⭐ W123：`LOG_TARGET` 的"只报一次"标志（首次收敛）
  bool converged_reported_ = false;
  int switch_count_ = 0;
  int update_count_ = 0;

  bool is_switch_ = false, is_converged_ = false;

  tools::ExtendedKalmanFilter ekf_;
  std::chrono::steady_clock::time_point t_;

  void update_ypda(const Armor & armor, int id);  // yaw pitch distance angle

  Eigen::Vector3d h_armor_xyz(const Eigen::VectorXd & x, int id) const;
  Eigen::MatrixXd h_jacobian(const Eigen::VectorXd & x, int id) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TARGET_HPP