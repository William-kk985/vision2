/**
 * @file core/debug.hpp
 * @brief ⭐ Debug 数据面：各角色的 Debug 结构体 + FrameDebug 聚合
 *
 * 设计原则（doc 09 §14.1）：
 *   ⭐ **L0~L2 常驻、零宏**（~ns 级成本，永远开）；**只有 L3 昂贵通道用宏**
 *   这样"算法 .cpp 里不出现 #ifdef"这条纪律才成立。
 *
 * 级别：
 *   L0 耗时/帧号/模式      ~100 ns/帧   永远开
 *   L1 每角色一个快照结构  ~50 ns/帧    永远开
 *   L2 多帧序列（曲线）     ~50 ns/项    永远开（走 IDebugSink::on_series）
 *   L3 存图/存点云          ~ms          ⚠️ 唯一用宏的（DEBUG_L3_ENABLE）
 *
 * ⚠️ 位置说明：doc 09 原写"Debug 结构体跟角色走"，但 FrameDebug 要聚合全部 →
 *    会形成角色间循环依赖。**改为集中放契约层**（它们都是纯数据，本就属于契约）。
 */
#ifndef HZMIR_CORE_DEBUG_HPP
#define HZMIR_CORE_DEBUG_HPP

#include <cstdint>

#include "core/types.hpp"

namespace auto_aim
{

/// 识别器
struct DetectorDebug
{
  int armor_count = 0;
  double best_confidence = 0;
  int nms_survivors = 0;
  int64_t t_infer_us = 0;   // ⭐ L0 本段耗时（学哈工程 DebugExpense）
};

/// 坐标变换器
struct SolverDebug
{
  int solved = 0;
  double reprojection_error = 0;
  double yaw_offset = 0;    // optimize_yaw 的结果
  int64_t t_solve_us = 0;
};

/// 跟踪器
struct TrackerDebug
{
  int state = 0;            // 0=lost 1=detecting 2=tracking 3=temp_lost
  int armor_count = 0;
  int priority_mode = 0;    // ⭐ W7
  int filtered_out = 0;     // ⭐ W7 被射击过滤器滤掉几块
  int invincible_count = 0; // ⭐ W35 ROS2 下行：当前已知无敌敌人个数
  int focus_target_count = 0;  // ⭐ W35 ROS2 下行：集火指令指定了几个目标
  int64_t t_track_us = 0;
};

/// 估计器
struct TargetDebug
{
  int tracked_id = 0;
  double xyz_world[3] = {0, 0, 0};
  double yaw = 0, w = 0, r = 0, l = 0, h = 0;
  double nis = 0, nis_thresh = 0;   // ⭐ 滤波器健康度
  bool invincible = false;          // ⭐ W7
  int64_t t_update_us = 0;
};

/// 规划器
struct PlannerDebug
{
  double t_fly = 0, t_fire = 0, t_pred = 0;   // 同济理论三个时间
  double overlap_ratio = 0, dps = 0, kill_time = 0;  // ⭐ 重合度 / 秒伤 / 击杀时间
  int solver_iters = 0;
  double acc_max = 0;
  int64_t t_plan_us = 0;
};

/// 开火器
struct ShooterDebug
{
  double traj_err_at_fire = 0, fire_thresh = 0;
  bool should_fire = false;
  bool blocked_by_invincible = false;   // ⭐ W7
  bool blocked_by_filter = false;       // ⭐ W7
  int64_t t_since_last_fire_us = 0;
};

/// 控制器
struct ControllerDebug
{
  double cmd_yaw = 0, cmd_pitch = 0;
  bool control = false, shoot = false;
  int64_t t_ctrl_us = 0;
};

/// 打符（简单版，够 W8 用）
struct BuffDebug
{
  int rune_type = 0;      // 0=小符 1=大符
  int fanblade_count = 0;
  double spd = 0;
  bool solved = false;
  int64_t t_us = 0;
};

/// ⭐ 一帧的完整调试快照（L0 + L1）
struct FrameDebug
{
  // ── L0 常驻 ──
  int64_t t_frame_us = 0;
  int64_t t_perceive_us = 0;
  int64_t t_decide_us = 0;
  uint32_t frame_id = 0;
  uint8_t mode = 0;
  uint8_t game_state = 0;

  // ── L1 快照（各角色自带）──
  DetectorDebug detector;
  SolverDebug solver;
  TrackerDebug tracker;
  TargetDebug target;
  PlannerDebug planner;
  ShooterDebug shooter;
  ControllerDebug controller;
  BuffDebug buff;
};

}  // namespace auto_aim

#endif  // HZMIR_CORE_DEBUG_HPP
