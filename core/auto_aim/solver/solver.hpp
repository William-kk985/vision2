#ifndef AUTO_AIM__SOLVER_HPP
#define AUTO_AIM__SOLVER_HPP

#include <Eigen/Dense>  // 必须在opencv2/core/eigen.hpp上面
#include <Eigen/Geometry>
#include <opencv2/core/eigen.hpp>

#include "core/auto_aim/solver/solver_debug.hpp"   // ⭐ W71
#include "core/types.hpp"

namespace auto_aim
{
class Solver
{
public:
  explicit Solver(const std::string & config_path);

  Eigen::Matrix3d R_gimbal2world() const;

  void set_R_gimbal2world(const Eigen::Quaterniond & q);

  void solve(Armor & armor) const;

  /// ⭐⭐ W71：**上一帧 `solve()` 的统计** —— 解决 `sol_*` 四列永远是 0
  ///   （W67 排查：`fd.solver.*` 在 `src/*.cpp` 里**零赋值**）
  ///   ⚠️ `solve()` 是 `const` → 统计用 `mutable`。单线程（plan/perceive）访问，无需锁。
  auto_aim::SolverDebug last_debug() const { return last_dbg_; }

  std::vector<cv::Point2f> reproject_armor(
    const Eigen::Vector3d & xyz_in_world, double yaw, ArmorType type, ArmorName name) const;

  double oupost_reprojection_error(Armor armor, const double & picth);

  std::vector<cv::Point2f> world2pixel(const std::vector<cv::Point3f> & worldPoints);

private:
  cv::Mat camera_matrix_;
  cv::Mat distort_coeffs_;
  Eigen::Matrix3d R_gimbal2imubody_;
  Eigen::Matrix3d R_camera2gimbal_;
  Eigen::Vector3d t_camera2gimbal_;
  Eigen::Matrix3d R_gimbal2world_;

  void optimize_yaw(Armor & armor) const;

  mutable auto_aim::SolverDebug last_dbg_;   // ⭐ W71：solve() 是 const，故 mutable

  double armor_reprojection_error(const Armor & armor, double yaw, const double & inclined) const;
  double SJTU_cost(
    const std::vector<cv::Point2f> & cv_refs, const std::vector<cv::Point2f> & cv_pts,
    const double & inclined) const;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__SOLVER_HPP