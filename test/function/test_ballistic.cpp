// test/function/test_ballistic.cpp —— ⭐ W12：两个弹道实现的精度 vs 速度
//
// 回答两个问题：
//   ① 精度：`ideal`（斜抛无阻力，同济原版） vs `rk4_drag`（RK4 含阻力）差多少？
//   ② 速度：`rk4_drag` 能不能塞进 10 ms 帧预算？
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include "core/auto_aim/trajectory/trajectory.hpp"
#include "utils/wheels/ballistic/rk4_ballistic.hpp"

using namespace auto_aim;
using clock_t_ = std::chrono::steady_clock;

static double us_per(clock_t_::time_point a, clock_t_::time_point b, int n)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count() / 1000.0 / n;
}

int main()
{
  auto ideal = make_trajectory("ideal");
  auto rk4 = make_trajectory("rk4_drag");
  std::printf("实现: %s  vs  %s\n\n", ideal->name(), rk4->name());

  // ═══ ① 精度对比（步兵 17mm，初速 22 m/s）═══
  std::printf("① 精度对比（17mm 弹，v0=22 m/s，目标等高 h=0）\n");
  std::printf("   %8s %14s %14s %12s %12s\n", "距离(m)", "ideal pitch(°)", "rk4 pitch(°)",
              "Δpitch(mrad)", "Δt_fly(ms)");
  const double v0 = 22.0;
  double max_dp = 0, max_dt = 0;
  for (double d : {2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0}) {
    auto a = ideal->solve(v0, d, 0.0);
    auto b = rk4->solve(v0, d, 0.0);
    if (a.unsolvable || b.unsolvable) { std::printf("   %8.1f  (无解)\n", d); continue; }
    const double dp = (b.pitch - a.pitch) * 1000;              // mrad
    const double dt = (b.fly_time - a.fly_time) * 1000;        // ms
    max_dp = std::max(max_dp, std::abs(dp));
    max_dt = std::max(max_dt, std::abs(dt));
    std::printf("   %8.1f %14.4f %14.4f %12.3f %12.3f\n", d, a.pitch * 57.2958,
                b.pitch * 57.2958, dp, dt);
  }
  std::printf("   ⇒ 最大差: Δpitch = %.2f mrad (%.3f°)  Δt_fly = %.2f ms\n", max_dp,
              max_dp / 1000 * 57.2958, max_dt);

  // ═══ ② 英雄 42mm（阻力影响大得多）═══
  std::printf("\n② 英雄 42mm（v0=16 m/s，目标等高）\n");
  auto rk4_42 = make_trajectory("rk4_drag_42");
  std::printf("   %8s %14s %14s %12s %12s\n", "距离(m)", "ideal(°)", "rk4_42(°)", "Δpitch(mrad)",
              "Δt_fly(ms)");
  double max_dp42 = 0;
  for (double d : {3.0, 5.0, 8.0, 10.0, 15.0}) {
    auto a = ideal->solve(16.0, d, 0.0);
    auto b = rk4_42->solve(16.0, d, 0.0);
    if (a.unsolvable || b.unsolvable) { std::printf("   %8.1f  (无解)\n", d); continue; }
    const double dp = (b.pitch - a.pitch) * 1000;
    max_dp42 = std::max(max_dp42, std::abs(dp));
    std::printf("   %8.1f %14.4f %14.4f %12.3f %12.3f\n", d, a.pitch * 57.2958,
                b.pitch * 57.2958, dp, (b.fly_time - a.fly_time) * 1000);
  }
  std::printf("   ⇒ 最大差: Δpitch = %.2f mrad (%.3f°)\n", max_dp42, max_dp42 / 1000 * 57.2958);

  // ═══ ③ ⭐ 速度对比（决定能不能上赛场）═══
  std::printf("\n③ ⭐ 速度（每帧要算 1 次）\n");
  const int N = 2000;
  double acc = 0;
  auto t1 = clock_t_::now();
  for (int i = 0; i < N; ++i) acc += ideal->solve(v0, 3.0 + (i % 50) * 0.1, 0.0).pitch;
  auto t2 = clock_t_::now();
  const double ideal_us = us_per(t1, t2, N);

  auto t3 = clock_t_::now();
  for (int i = 0; i < N; ++i) acc += rk4->solve(v0, 3.0 + (i % 50) * 0.1, 0.0).pitch;
  auto t4 = clock_t_::now();
  const double rk4_us = us_per(t3, t4, N);

  auto t5 = clock_t_::now();
  for (int i = 0; i < N; ++i) acc += rk4_42->solve(16.0, 3.0 + (i % 50) * 0.1, 0.0).pitch;
  auto t6 = clock_t_::now();
  const double rk4_42_us = us_per(t5, t6, N);

  std::printf("   %-24s %10.2f us   占 10ms 帧预算 %.3f%%\n", "ideal (同济原版)", ideal_us,
              ideal_us / 10000 * 100);
  std::printf("   %-24s %10.2f us   占 10ms 帧预算 %.3f%%\n", "rk4_drag (17mm)", rk4_us,
              rk4_us / 10000 * 100);
  std::printf("   %-24s %10.2f us   占 10ms 帧预算 %.3f%%\n", "rk4_drag_42 (42mm)", rk4_42_us,
              rk4_42_us / 10000 * 100);
  std::printf("   ⇒ rk4 比 ideal 慢 %.0f 倍\n", rk4_us / ideal_us);
  (void)acc;

  // ═══ ④ 结论 ═══
  // ⭐ W30：步长对精度/速度的影响（同济 h_solver 用 dt=1e-2；原实现 1e-4）
  std::printf("\n⑤ ⭐ W30：RK4 步长的影响（同济 `h_solver` 用 dt=1e-2）\n");
  {
    auto make = [&] {
      return std::make_unique<tools::BallisticSolver>(tools::BulletType::SMALL_17MM);
    };
    // 以 dt=1e-4 为精度基准
    auto ref = make();
    ref->set_dt(1e-4);
    const double d = 6.0, v = 22.0;
    const auto r_ref = ref->solve(v, d, 0.0);

    std::printf("     %-10s %14s %14s %12s %12s\n", "dt", "pitch(°)", "fly_time(s)", "Δpitch(µrad)",
                "耗时(µs)");
    for (double dt : {1e-4, 5e-4, 1e-3, 5e-3, 1e-2}) {
      auto s2 = make();
      s2->set_dt(dt);
      const int M = 300;
      auto t0 = clock_t_::now();
      double acc2 = 0;
      for (int i = 0; i < M; ++i) acc2 += s2->solve(v, 3.0 + (i % 40) * 0.1, 0.0).pitch;
      auto t1 = clock_t_::now();
      (void)acc2;
      const auto rr = s2->solve(v, d, 0.0);
      std::printf("     %-10.0e %14.4f %14.5f %12.2f %12.1f\n", dt, rr.pitch * 57.2958,
                  rr.fly_time, (rr.pitch - r_ref.pitch) * 1e6, us_per(t0, t1, M));
    }
    std::printf("     ⇒ 步长放大 10 倍（1e-4→1e-3）应保持 µrad 级一致，但快约 10 倍\n");
  }

  std::printf("\n─ 判读 ──────────────────────────────────────────\n");
  std::printf("① 17mm 下两模型差 %.2f mrad (%.3f°)，而 RM 装甲板高 0.125 m、\n", max_dp,
              max_dp / 1000 * 57.2958);
  std::printf("   在 4 m 处视野张角约 %.1f mrad → %s\n", 125.0 / 4.0,
              max_dp < 31.25 / 2 ? "差异在半个装甲板以内" : "差异超过半个装甲板");
  std::printf("② 42mm 下差 %.2f mrad (%.3f°) → %s\n", max_dp42, max_dp42 / 1000 * 57.2958,
              max_dp42 > max_dp * 3 ? "⭐ 阻力影响**显著更大**（44g 高尔夫球确实需要）"
                                    : "与 17mm 量级相近");
  std::printf("③ 速度：rk4 占帧预算 %.2f%% → %s\n", rk4_us / 10000 * 100,
              rk4_us > 1000 ? "⚠️ 超过 1 ms，需评估" : "✅ 完全可接受");
  return 0;
}
