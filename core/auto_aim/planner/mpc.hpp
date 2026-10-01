#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>

#include <memory>

#include <yaml-cpp/yaml.h>

#include "core/auto_aim/target/target.hpp"
#include "core/auto_aim/trajectory/trajectory.hpp"   // ⭐ W15：弹道槽位（yaml 选实现）
#include "third_party/tinympc/tiny_api.hpp"

namespace auto_aim
{
constexpr double DT = 0.01;
constexpr int HALF_HORIZON = 50;
constexpr int HORIZON = HALF_HORIZON * 2;

using Trajectory = Eigen::Matrix<double, 4, HORIZON>;  // yaw, yaw_vel, pitch, pitch_vel

struct Plan
{
  bool control;
  bool fire;
  float target_yaw;
  float target_pitch;
  float yaw;
  float yaw_vel;
  float yaw_acc;
  float pitch;
  float pitch_vel;
  float pitch_acc;

  // ═══════════════════════════════════════════════════════════════
  // ⭐ W8 新增：**暴露算法内部量**（「算法上调试」的缺口）
  //
  // 原来这些量只存在于 `plan()` 的局部作用域里（`bullet_traj` / `traj` / 判据中间量），
  // 外部**完全看不到** → 无法回答"小陀螺打不中是估计不准还是规划不对"。
  // 加默认值 = 向后兼容（`buff_aimer` 等现有调用点零改动）。
  // ═══════════════════════════════════════════════════════════════
  float t_fly = 0;            // 弹丸飞行时间（同济理论的 t_fly）
  float bullet_pitch = 0;     // 弹道仰角
  float min_dist = 0;         // 最近装甲板水平距离
  bool  unsolvable = false;   // 弹道无解（delta < 0）
  float traj_err = 0;         // ⭐ **开火判据量**：前瞻点的「参考轨迹 vs 规划轨迹」误差
  float fire_thresh = 0;      // 判据阈值（yaml 的 fire_thresh_）
  float overlap = 0;          // ⭐ 前瞻段重合度 = (1 - traj_err/阈值) 截断到 [0,1]
  int   yaw_iters = 0;        // yaw solver 迭代次数
  int   pitch_iters = 0;      // pitch solver 迭代次数
  float acc_max = 0;          // 规划轨迹最大加速度（看有没有超物理上限）
};

class Planner
{
public:
  Eigen::Vector4d debug_xyza;
  Planner(const std::string & config_path);

  /// ⭐ W21：热重载标定/阈值（按键 `r`）——**不重建 MPC 求解器**（重建要几十 ms）
  void reload(const YAML::Node & yaml);

  Plan plan(Target target, double bullet_speed);
  Plan plan(std::optional<Target> target, double bullet_speed);

private:
  double yaw_offset_;
  double pitch_offset_;
  double fire_thresh_;
  double low_speed_delay_time_, high_speed_delay_time_, decision_speed_;

  TinySolver * yaw_solver_;
  TinySolver * pitch_solver_;

  // ⭐ W15：弹道实现（yaml 的 `trajectory_impl` 选；默认 "ideal"）
  //   可选项：ideal / rk4_drag / table / table_42 / tongji_linear / ...
  std::unique_ptr<ITrajectory> trajectory_;

  void setup_yaw_solver(const std::string & config_path);
  void setup_pitch_solver(const std::string & config_path);

  Eigen::Matrix<double, 2, 1> aim(const Target & target, double bullet_speed);
  Trajectory get_trajectory(Target & target, double yaw0, double bullet_speed);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PLANNER_HPP