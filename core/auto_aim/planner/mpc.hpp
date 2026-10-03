#ifndef AUTO_AIM__PLANNER_HPP
#define AUTO_AIM__PLANNER_HPP

#include <Eigen/Dense>
#include <list>
#include <optional>

#include <memory>

#include <yaml-cpp/yaml.h>

#include "core/auto_aim/controller/controller_debug.hpp"   // ⭐ W98
#include "core/auto_aim/planner/planner_debug.hpp"          // ⭐ W98
#include "core/auto_aim/shooter/shooter_debug.hpp"          // ⭐ W98
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

  // ⭐⭐⭐ W113：**瞄准点的世界坐标 (x, y, z, yaw)** —— 从 `Planner::debug_xyza` 带进来
  //   ⚠️ 为什么必须进 `Plan`：`Planner::debug_xyza` 在 **plan 线程**里写，
  //     主循环直接读它 = **数据竞争**。`Plan` 随 `psnap`（带锁）传递 ⇒ 安全。
  //   ⭐ 用途：主循环 `reproject_armor()` 把它投回像素 ⇒ **画旧版赫兹那个"红圈瞄准点"**。
  //   ⭐ 默认零向量 ⇒ 主循环判 `norm() > 1e-6` 即知"没填"（不需额外标志位）。
  Eigen::Vector4d debug_xyza = Eigen::Vector4d::Zero();

  // ⭐⭐ W98：**把本 Plan 的调试量路由到三个角色的 Debug 结构**
  //
  // ## 为什么放在这里（而不是各主循环手写）
  // 原来 **4 个主循环各自手写 10 行**把 `Plan` 的字段拆到 `fd.planner` /
  // `fd.shooter` / `fd.controller`：
  // ```
  // const auto & p = psnap.plan;
  // fd.planner.t_fly = p.t_fly;              // ⚠️ ×4 份
  // fd.planner.overlap_ratio = p.overlap;
  // fd.shooter.traj_err_at_fire = p.traj_err;
  // fd.controller.cmd_yaw = p.yaw;
  // ...
  // ```
  // ⇒ 加一个新的调试量要改 **4 处**；漏一处就某个兵种看不到（W8/W73 都踩过）。
  // ⇒ 现在**映射只有这一份**，且**贴着产它的算法**（改 `Plan` 的人一眼能看到）。
  //
  // ⚠️ 只填**来自 Plan 的**字段。主循环独有的（`blocked_by_invincible` /
  //   `blocked_by_filter` / `t_since_last_fire_us` / `t_track_us` …）**不在这里**，
  //   避免把主循环的状态覆盖掉。
  //
  // @param planner    `PlannerDebug`（t_fly / overlap_ratio / solver_iters / acc_max）
  // @param shooter    `ShooterDebug`（traj_err_at_fire / fire_thresh / should_fire）
  // @param controller `ControllerDebug`（cmd_yaw / cmd_pitch / control / shoot）
  void fill_debug(
    PlannerDebug & planner, ShooterDebug & shooter, ControllerDebug & controller) const
  {
    planner.t_fly = t_fly;
    planner.overlap_ratio = overlap;
    planner.solver_iters = yaw_iters;
    planner.acc_max = acc_max;

    shooter.traj_err_at_fire = traj_err;
    shooter.fire_thresh = fire_thresh;
    shooter.should_fire = fire;

    controller.cmd_yaw = yaw;
    controller.cmd_pitch = pitch;
    controller.control = control;
    controller.shoot = fire;
  }
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