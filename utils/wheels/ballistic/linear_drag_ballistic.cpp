#include "utils/wheels/ballistic/linear_drag_ballistic.hpp"

#include <cmath>

namespace tools
{

LinearDragBallistic::LinearDragBallistic(bool use_analytical) : analytical_(use_analytical)
{
  // 同济原式：k1 = (Cd · ρ · π · r²) / (2m)   ← 即 0.5·Cd·ρ·A/m
  k1_ = (TONGJI_CD * TONGJI_RHO * M_PI * TONGJI_RADIUS * TONGJI_RADIUS) / (2 * TONGJI_MASS);
}

void LinearDragBallistic::set_params(double k1, double g)
{
  k1_ = k1;
  g_ = g;
}

BallisticResult LinearDragBallistic::solve(double v0, double d, double h) const
{
  BallisticResult result;
  if (d < 1e-6) return result;                 // 同济：距离过小视为无解
  if (v0 <= 0) return result;

  double theta = std::atan(h / d);             // 同济的初值
  double fly_time = 0;

  for (int i = 0; i < MAX_ITER; ++i) {
    const double ct = std::cos(theta);
    const double st = std::sin(theta);
    if (std::abs(ct) < 1e-9) break;

    // ── 飞行时间：两种形式 ──
    const double kd = k1_ * d;
    if (analytical_) {
      // ⭐ 同济原式（指数形式）
      fly_time = (std::exp(kd) - 1.0) / (k1_ * v0 * ct);
    } else {
      // ⭐ 线性阻力下的正确解析式（对数形式）
      const double denom = 1.0 - kd / (v0 * ct);
      if (denom <= 0) { result.solved = false; return result; }   // 打不到
      fly_time = -std::log(denom) / k1_;
    }

    // 高度误差
    const double delta_z =
      h - v0 * st * fly_time / ct + 0.5 * g_ * fly_time * fly_time / (ct * ct);
    if (std::abs(delta_z) < TOL) break;

    // 牛顿步（同济原式的导数）
    const double ct2 = ct * ct;
    const double denom =
      -(v0 * fly_time) / ct2 + g_ * fly_time * fly_time / (v0 * v0) * st / (ct2 * ct);
    if (std::abs(denom) < 1e-12) break;
    theta -= delta_z / denom;
  }

  result.solved = true;
  result.pitch = theta;
  result.fly_time = fly_time;
  result.iterations = MAX_ITER;   // 原实现没记迭代次数；这里不谎报，测试另测
  return result;
}

}  // namespace tools
