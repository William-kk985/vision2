/**
 * @file utils/wheels/ballistic/rk4_ballistic.hpp
 * @brief ⭐ 轮子：含空气阻力的弹道解算（四阶龙格-库塔）
 *
 * **来源**：`Hz_Rm_Vision/src/utils/math/ballistic_solver.{hpp,cpp}`（你的实现，321 行）
 * **落位理由**：`grep` 证明它**零业务依赖**（只需 `<cmath>`）→ 属于 `utils/wheels/`
 * **适配**：`core/auto_aim/trajectory/rk4_drag.cpp` 把它接成 `ITrajectory`
 *
 * 运动方程（`state = [x, y, vx, vy]`）：
 *   dvx/dt = -k·|v|·vx
 *   dvy/dt = -g -k·|v|·vy
 *   k = 0.5·ρ·Cd·A / m
 *
 * ⚠️ **移植时修的坑**：原头文件**缺 `#include <array>`**（靠传递包含侥幸编过 —— 同 C23 一类）
 * ⚠️ 原文注释有不可逆的编码损坏，本文件按原意重写
 */
#ifndef HZMIR_UTILS_WHEELS_BALLISTIC_RK4_BALLISTIC_HPP
#define HZMIR_UTILS_WHEELS_BALLISTIC_RK4_BALLISTIC_HPP

#include <array>   // ⭐ 移植修复：原文件缺这个 include
#include <cmath>
#include <utility>
#include <vector>

namespace tools
{

/// 弹丸类型（RM 官方参数）
enum class BulletType
{
  SMALL_17MM,  ///< 17mm 小弹丸（步兵 / 哨兵）
  BIG_42MM     ///< 42mm 大弹丸（英雄，凹坑高尔夫球）
};

struct BallisticResult
{
  bool solved = false;    ///< 是否求解成功
  double pitch = 0;       ///< 发射仰角（rad，抬头为正）
  double fly_time = 0;    ///< 飞行时间（s）
  double drop = 0;        ///< 相对无阻力模型的额外下坠（m）
  int iterations = 0;     ///< 二分迭代次数
};

class BallisticSolver
{
public:
  explicit BallisticSolver(BulletType bullet_type = BulletType::SMALL_17MM);

  /// 自定义弹丸参数（覆盖官方值）
  void set_bullet_params(double mass, double diameter, double drag_coeff);

  /// ⭐ 设置 RK4 步长（越大越快、越不准；默认 `DEFAULT_DT = 1e-3`）
  void set_dt(double dt) { dt_ = dt > 0 ? dt : DEFAULT_DT; }
  double dt() const { return dt_; }

  /// 求解；@param use_air_resistance false 时退化为 `solve_simple`
  BallisticResult solve(
    double v0, double distance, double height, bool use_air_resistance = true) const;

  /// 无阻力抛物线（解析解，⭐ 微秒级）
  BallisticResult solve_simple(double v0, double distance, double height) const;

  /// 含阻力（RK4 + 二分，⚠️ **毫秒级**，见测试实测）
  BallisticResult solve_rk4(double v0, double distance, double height) const;

  /// 给定仰角正推轨迹；@return 是否到达 target_x
  bool simulate_trajectory(
    double v0, double pitch, double target_x, double & final_y, double & fly_time) const;

  /// 完整轨迹点（可视化用）
  std::vector<std::pair<double, double>> get_trajectory_points(
    double v0, double pitch, double target_x, int num_points = 50) const;

  // 物理常数（可读，便于外部核对）
  static constexpr double G = 9.7833;      ///< 重力加速度（m/s²）
  static constexpr double RHO = 1.204;     ///< 空气密度（kg/m³ @20°C）

private:
  void update_k();
  std::array<double, 4> rk4_step(const std::array<double, 4> & state, double dt) const;
  std::array<double, 4> derivatives(const std::array<double, 4> & state) const;

  double dt_ = DEFAULT_DT;   ///< ⭐ 可配步长
  double mass_ = 0, diameter_ = 0, drag_coeff_ = 0;
  double k_ = 0;   ///< 综合阻力系数 = 0.5·ρ·Cd·A/m

  // ⭐⭐ 步长与终止条件（W30：由同济 `lob_shoot_hero` 的 `h_solver` 交叉印证）
  //   同济 `h_solver::RungeKutta_4` 用 **`dt = 0.01`**、`while (y >= 0 && t < max_time)`（**落地即停**）；
  //   本实现原先用 `dt = 1e-4` + 固定 `MAX_TIME = 3 s` → 单次求解最多 30000 步（建表 16 s 的根因）。
  //   ⭐ 现在：步长**可配**（默认 1e-3，介于两者之间）+ **落地即停**。
  static constexpr double DEFAULT_DT = 0.001;   ///< 默认步长（原 1e-4；同济 h_solver 用 1e-2）
  static constexpr double MAX_TIME = 3.0;       ///< 最大飞行时间（s，兜底）
  static constexpr double GROUND_Y = -10.0;     ///< 落地判据（y < 此值视为无效）
  static constexpr int MAX_ITER = 50;       ///< 二分最大迭代
  static constexpr double TOL = 0.001;      ///< 收敛容差（m）
};

}  // namespace tools

#endif  // HZMIR_UTILS_WHEELS_BALLISTIC_RK4_BALLISTIC_HPP
