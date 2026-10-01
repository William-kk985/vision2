// test/function/test_ballistic_table.cpp —— ⭐ W13：弹道表能不能做到「RK4 精度 + ideal 速度」
//
// 验证四件事：
//   ① 建表成本（一次性）与表体积
//   ② 精度：table vs rk4（h=0，建表高度）
//   ③ ⭐ **假设检验**：修正量对 h 弱相关吗？（h≠0 时还准不准）
//   ④ 速度：table 是否回到 ideal 量级
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>

#include "core/auto_aim/trajectory/trajectory.hpp"

using namespace auto_aim;
using clk = std::chrono::steady_clock;

static double us_per(clk::time_point a, clk::time_point b, int n)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count() / 1000.0 / n;
}

int main()
{
  auto ideal = make_trajectory("ideal");
  auto rk4 = make_trajectory("rk4_drag");
  auto rk4_42 = make_trajectory("rk4_drag_42");

  std::printf("① 建表（离线一次性）\n");
  auto t1 = clk::now();
  auto table = make_trajectory("table");
  auto t2 = clk::now();
  auto table_42 = make_trajectory("table_42");
  auto t3 = clk::now();
  std::printf("   17mm 表建表耗时 %.1f ms\n",
              std::chrono::duration<double>(t2 - t1).count() * 1000);
  std::printf("   42mm 表建表耗时 %.1f ms\n",
              std::chrono::duration<double>(t3 - t2).count() * 1000);
  std::printf("   轴: v0 10~25 step .5 (31) × d 1~15 step .1 (141) = 4371 点\n");

  // ═══ ② 精度：table vs rk4（h = 0）═══
  std::printf("\n② 精度 table vs rk4（h=0，即建表高度）\n");
  std::printf("   %8s %14s %14s %14s\n", "距离(m)", "rk4(°)", "table(°)", "误差(mrad)");
  double max_err = 0;
  for (double d : {1.5, 2.0, 3.7, 5.0, 6.3, 8.0, 10.0, 12.5, 14.9}) {
    const auto a = rk4->solve(22.0, d, 0.0);
    const auto b = table->solve(22.0, d, 0.0);
    const double e = (b.pitch - a.pitch) * 1000;
    max_err = std::max(max_err, std::abs(e));
    std::printf("   %8.1f %14.4f %14.4f %14.4f\n", d, a.pitch * 57.2958, b.pitch * 57.2958, e);
  }
  std::printf("   ⇒ 最大插值误差 %.4f mrad\n", max_err);

  // ═══ ③ ⭐ 假设检验：h ≠ 0 时还准吗？═══
  std::printf("\n③ ⭐ 假设检验：修正量对 h 弱相关？（v0=22, d=6 m）\n");
  std::printf("   %10s %14s %14s %14s\n", "目标高(m)", "rk4(°)", "table(°)", "误差(mrad)");
  double max_err_h = 0;
  for (double h : {-1.0, -0.5, 0.0, 0.5, 1.0, 2.0}) {
    const auto a = rk4->solve(22.0, 6.0, h);
    const auto b = table->solve(22.0, 6.0, h);
    if (a.unsolvable || b.unsolvable) { std::printf("   %10.1f  (无解)\n", h); continue; }
    const double e = (b.pitch - a.pitch) * 1000;
    max_err_h = std::max(max_err_h, std::abs(e));
    std::printf("   %10.1f %14.4f %14.4f %14.4f\n", h, a.pitch * 57.2958, b.pitch * 57.2958, e);
  }
  std::printf("   ⇒ h ∈ [-1,2] m 时最大误差 %.4f mrad → %s\n", max_err_h,
              max_err_h < 2.0 ? "✅ 假设成立（误差可忽略）" : "⚠️ 假设不成立，需改用 3D 表");

  // ═══ ④ 速度 ═══
  const int N = 2000;
  double acc = 0;
  auto s1 = clk::now();
  for (int i = 0; i < N; ++i) acc += ideal->solve(22.0, 3.0 + (i % 50) * 0.1, 0.0).pitch;
  auto s2 = clk::now();
  for (int i = 0; i < N; ++i) acc += rk4->solve(22.0, 3.0 + (i % 50) * 0.1, 0.0).pitch;
  auto s3 = clk::now();
  for (int i = 0; i < N; ++i) acc += table->solve(22.0, 3.0 + (i % 50) * 0.1, 0.0).pitch;
  auto s4 = clk::now();
  (void)acc;

  const double ideal_us = us_per(s1, s2, N), rk4_us = us_per(s2, s3, N), tab_us = us_per(s3, s4, N);
  std::printf("\n④ ⭐ 速度（每帧 1 次）\n");
  std::printf("   %-22s %10.2f us   占 10ms 帧预算 %.4f%%\n", "ideal", ideal_us,
              ideal_us / 10000 * 100);
  std::printf("   %-22s %10.2f us   占 10ms 帧预算 %.3f%%\n", "rk4_drag", rk4_us,
              rk4_us / 10000 * 100);
  std::printf("   %-22s %10.2f us   占 10ms 帧预算 %.4f%%  ⭐\n", "table（新）", tab_us,
              tab_us / 10000 * 100);
  std::printf("   ⇒ table 比 rk4 快 %.0f 倍，只比 ideal 慢 %.1f 倍\n", rk4_us / tab_us,
              tab_us / ideal_us);

  std::printf("\n─ 结论 ─────────────────────────────────────────\n");
  const bool reach = (max_err < 1.0) && (max_err_h < 2.0) && (tab_us < rk4_us / 100);
  std::printf("%s 「离线 RK4 建表 + 在线查表」达成目标：\n",
              reach ? "✅" : "⚠️");
  std::printf("   精度：与 rk4 差 %.4f mrad（h 变化时不超 %.4f mrad）\n", max_err, max_err_h);
  std::printf("   速度：%.2f us vs rk4 的 %.2f us（快 %.0f 倍）\n", tab_us, rk4_us, rk4_us / tab_us);
  return 0;
}
