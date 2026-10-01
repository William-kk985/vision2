/**
 * @file utils/wheels/ballistic/linear_drag_ballistic.hpp
 * @brief ⭐ 轮子：**同济英雄分支**的弹道模型（线性阻力解析解 + 牛顿迭代）
 *
 * **来源**：同济 `origin/auto_aim_hero` 的 `tools/trajectory.cpp`（`Trajectory(v0,d,h,mode)`，mode=1）
 * **与你的 `rk4_ballistic` 的根本区别**：
 *
 * | | 阻力模型 | 求解方式 | 代价 |
 * |---|---|---|---|
 * | 你的 `rk4_ballistic` | **二次** `f = k·v²` | RK4 积分 + 二分 | 1328 µs |
 * | **同济英雄（本文件）** | **线性** `f = k1·v` | **解析解 + 牛顿迭代** | 待测（预期 µs 级） |
 *
 * 线性阻力下水平方向可解析积分：
 * ```
 *   x(t) = v0·cosθ · (1 - e^(-k1·t)) / k1
 *   ⇒ t(x) = -ln(1 - k1·x / (v0·cosθ)) / k1
 * ```
 * 同济写成了等价的 `t = (e^(k1·x) - 1) / (k1·v0·cosθ)`（⚠️ 见下方注释的存疑点）。
 *
 * ⚠️ **存疑点（移植时保留原样，供测试判定）**：
 *   原式 `t = (e^{k1·d} - 1)/(k1·v0·cosθ)` 在 `k1·d` 稍大时会**指数爆炸**，
 *   而上面推导的正确形式含 `ln(1 - k1·d/(v0·cosθ))`。
 *   两者在 `k1·d → 0` 时一阶等价，但**大距离下会显著分歧** —— 由测试实测。
 */
#ifndef HZMIR_UTILS_WHEELS_BALLISTIC_LINEAR_DRAG_BALLISTIC_HPP
#define HZMIR_UTILS_WHEELS_BALLISTIC_LINEAR_DRAG_BALLISTIC_HPP

#include "utils/wheels/ballistic/rk4_ballistic.hpp"   // 复用 BallisticResult

namespace tools
{

/// 同济英雄分支的弹道模型（42mm 大弹丸）
class LinearDragBallistic
{
public:
  /// @param use_analytical  true = 同济原式（指数）；false = 正确解析式（对数）
  ///        ⭐ 提供两种是为了**实测哪个对**（见 test_ballistic_tongji）
  explicit LinearDragBallistic(bool use_analytical = true);

  /// 覆盖阻力系数 k1（默认用同济的 42mm 值）与重力
  void set_params(double k1, double g);

  BallisticResult solve(double v0, double d, double h) const;

  // ⭐ 同济英雄分支的常数（原样保留）
  static constexpr double TONGJI_G = 9.7946;         ///< 原 `g = 9.7946`（⚠️ main 是 9.7833）
  static constexpr double TONGJI_CD = 0.47;          ///< 球体阻力系数
  static constexpr double TONGJI_RHO = 1.169;        ///< 空气密度
  static constexpr double TONGJI_RADIUS = 0.02125;   ///< 弹丸半径（42mm）
  static constexpr double TONGJI_MASS = 0.041;       ///< 弹丸质量（42mm）
  static constexpr int MAX_ITER = 100;               ///< 牛顿迭代上限
  static constexpr double TOL = 1e-6;                ///< 收敛判据

private:
  double k1_ = 0;
  double g_ = TONGJI_G;
  bool analytical_ = true;
};

}  // namespace tools

#endif  // HZMIR_UTILS_WHEELS_BALLISTIC_LINEAR_DRAG_BALLISTIC_HPP
