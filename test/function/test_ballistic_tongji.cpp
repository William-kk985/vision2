// test/function/test_ballistic_tongji.cpp —— ⭐ W14：同济英雄的线性阻力模型到底行不行
//
// 五路对比（**以 rk4_drag_42 的二次阻力为物理基准**）：
//   ideal              无阻力解析
//   rk4_drag_42        二次阻力 + RK4（物理最正确）
//   table_42           二次阻力 + 查表（近似 rk4）
//   tongji_linear      ⭐ 同济英雄的线性阻力（指数式，原样）
//   tongji_linear_log  ⭐ 线性阻力 + 正确对数解析式（我的推导）
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>

#include "core/auto_aim/trajectory/trajectory.hpp"
#include "utils/wheels/ballistic/linear_drag_ballistic.hpp"

using namespace auto_aim;
using clk = std::chrono::steady_clock;

static double us_per(clk::time_point a, clk::time_point b, int n)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count() / 1000.0 / n;
}

int main()
{
  std::printf("同济英雄的 k1 = %.6f  (Cd·ρ·π·r²)/(2m) = (0.47·1.169·π·0.02125²)/(2·0.041)\n",
              tools::LinearDragBallistic::TONGJI_CD * tools::LinearDragBallistic::TONGJI_RHO *
                M_PI * tools::LinearDragBallistic::TONGJI_RADIUS *
                tools::LinearDragBallistic::TONGJI_RADIUS /
                (2 * tools::LinearDragBallistic::TONGJI_MASS));
  std::printf("同济 hero 的 g = %.4f  （⚠️ main 是 9.7833）\n\n",
              tools::LinearDragBallistic::TONGJI_G);

  auto ideal = make_trajectory("ideal");
  auto rk4 = make_trajectory("rk4_drag_42");
  auto table = make_trajectory("table_42");
  auto tj = make_trajectory("tongji_linear");
  auto tjlog = make_trajectory("tongji_linear_log");

  // ═══ ① 精度：与 RK4（物理基准）比 ═══
  const double v0 = 16.0;   // 英雄初速
  std::printf("① 精度对比（v0=16 m/s，h=0，基准 = rk4_drag_42 二次阻力）\n");
  std::printf("   %7s %11s %11s %11s %11s %11s\n", "d(m)", "rk4基准(°)", "Δideal", "Δtable",
              "Δ同济lin", "Δ同济log");
  double mx_ideal = 0, mx_table = 0, mx_tj = 0, mx_tjlog = 0;
  for (double d : {2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 15.0}) {
    const auto r = rk4->solve(v0, d, 0.0);
    const auto a = ideal->solve(v0, d, 0.0);
    const auto t = table->solve(v0, d, 0.0);
    const auto p = tj->solve(v0, d, 0.0);
    const auto q = tjlog->solve(v0, d, 0.0);
    if (r.unsolvable) { std::printf("   %7.1f  (rk4 无解)\n", d); continue; }

    // 单位 mrad
    auto mm = [&](const tools::Trajectory & x) {
      return x.unsolvable ? std::nan("") : (x.pitch - r.pitch) * 1000;
    };
    const double di = mm(a), dt = mm(t), dp = mm(p), dq = mm(q);
    if (std::isfinite(di)) mx_ideal = std::max(mx_ideal, std::abs(di));
    if (std::isfinite(dt)) mx_table = std::max(mx_table, std::abs(dt));
    if (std::isfinite(dp)) mx_tj = std::max(mx_tj, std::abs(dp));
    if (std::isfinite(dq)) mx_tjlog = std::max(mx_tjlog, std::abs(dq));

    std::printf("   %7.1f %11.4f %11.3f %11.3f %11.3f %11.3f\n", d, r.pitch * 57.2958, di, dt,
                dp, dq);
  }
  std::printf("   ⇒ 与基准的最大偏差(mrad):  ideal %.2f | table %.4f | 同济lin %.2f | 同济log %.2f\n",
              mx_ideal, mx_table, mx_tj, mx_tjlog);

  // ═══ ② 飞行时间偏差（对 MPC 预测更致命）═══
  std::printf("\n② 飞行时间 vs 基准（ms）\n");
  std::printf("   %7s %11s %11s %11s %11s\n", "d(m)", "rk4基准", "Δideal", "Δ同济lin", "Δ同济log");
  for (double d : {3.0, 5.0, 8.0, 12.0, 15.0}) {
    const auto r = rk4->solve(v0, d, 0.0);
    if (r.unsolvable) continue;
    auto ms = [&](const tools::Trajectory & x) {
      return x.unsolvable ? std::nan("") : (x.fly_time - r.fly_time) * 1000;
    };
    std::printf("   %7.1f %11.2f %11.2f %11.2f %11.2f\n", d, r.fly_time * 1000,
                ms(ideal->solve(v0, d, 0.0)), ms(tj->solve(v0, d, 0.0)),
                ms(tjlog->solve(v0, d, 0.0)));
  }

  // ═══ ③ 速度 ═══
  const int N = 3000;
  double acc = 0;
  auto t1 = clk::now();
  for (int i = 0; i < N; ++i) acc += ideal->solve(v0, 3.0 + (i % 60) * 0.2, 0.0).pitch;
  auto t2 = clk::now();
  for (int i = 0; i < N; ++i) acc += table->solve(v0, 3.0 + (i % 60) * 0.2, 0.0).pitch;
  auto t3 = clk::now();
  for (int i = 0; i < N; ++i) acc += tj->solve(v0, 3.0 + (i % 60) * 0.2, 0.0).pitch;
  auto t4 = clk::now();
  for (int i = 0; i < N; ++i) acc += tjlog->solve(v0, 3.0 + (i % 60) * 0.2, 0.0).pitch;
  auto t5 = clk::now();
  for (int i = 0; i < N; ++i) acc += rk4->solve(v0, 3.0 + (i % 60) * 0.2, 0.0).pitch;
  auto t6 = clk::now();
  (void)acc;

  std::printf("\n③ ⭐ 速度（每帧 1 次，42mm）\n");
  struct Row { const char * n; double us; };
  const Row rows[] = {
    {"ideal", us_per(t1, t2, N)},        {"table_42（二次+查表）", us_per(t2, t3, N)},
    {"tongji_linear（原式）", us_per(t3, t4, N)}, {"tongji_linear_log", us_per(t4, t5, N)},
    {"rk4_drag_42（二次+RK4）", us_per(t5, t6, N)}};
  for (const auto & r : rows)
    std::printf("   %-26s %10.2f us   占 10ms 帧预算 %.4f%%\n", r.n, r.us, r.us / 10000 * 100);

  std::printf("\n─ 判读 ────────────────────────────────────────────\n");
  std::printf("① 同济线性模型的物理正确性：与二次阻力基准最大差 %.1f mrad (%.2f°)\n", mx_tj,
              mx_tj / 1000 * 57.2958);
  std::printf("   而无阻力模型(ideal)差 %.1f mrad (%.2f°) → %s\n", mx_ideal,
              mx_ideal / 1000 * 57.2958,
              mx_tj < mx_ideal ? "✅ 线性阻力确实比无阻力更接近物理" : "❌ 反而更差");
  std::printf("② 同济原式 vs 正确对数式：%.1f vs %.1f mrad → %s\n", mx_tj, mx_tjlog,
              mx_tjlog < mx_tj * 0.5 ? "⭐ 原式在大距离下有偏差（指数爆炸）"
                                     : "两者接近（原式在常用距离内可用）");
  return 0;
}
