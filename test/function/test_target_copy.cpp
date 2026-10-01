/**
 * @file test/function/test_target_copy.cpp
 * @brief ⭐ W41：`Target` 拷贝成本**归因**（修正版）
 *
 * ⚠️ 第一版 benchmark 有缺陷：用 `vector::push_back` 测，把**对象构造 + 分配**也算进去了
 *    （证据：连「移动」都测出 647 ns，而移动本该 ~30 ns）。
 * 本版**只测纯拷贝构造**：`T dst(src);` 放在 `volatile` 里防优化。
 *
 * 要回答的问题：**5.7× 到底花在 `unique_ptr` 的 `new`，还是别的地方？**
 */
#include <chrono>
#include <cstdio>
#include <cstring>
#include <memory>

#include "core/auto_aim/target/target.hpp"
#include "core/auto_aim/target/target_plugins.hpp"
#include "utils/ekf/extended_kalman_filter.hpp"

using Clock = std::chrono::steady_clock;
static double ns_per(Clock::time_point a, Clock::time_point b, int n)
{
  return std::chrono::duration<double, std::nano>(b - a).count() / n;
}

static volatile size_t g_sink = 0;

template <class F>
static double bench(int n, F && f)
{
  auto t0 = Clock::now();
  for (int i = 0; i < n; ++i) f();
  auto t1 = Clock::now();
  return ns_per(t0, t1, n);
}

int main()
{
  constexpr int N = 200000;
  std::printf("═══ Target 拷贝成本归因 v2（W41）═══\n\n");
  std::printf("  sizeof(Target) = %zu B\n\n", sizeof(auto_aim::Target));

  // ── ① 纯 EKF 拷贝（Target 里最大的成员）──
  {
    Eigen::VectorXd x0 = Eigen::VectorXd::Zero(11);
    Eigen::MatrixXd P0 = Eigen::MatrixXd::Identity(11, 11);
    auto add = [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; };
    tools::ExtendedKalmanFilter ekf(x0, P0, add);
    double t = bench(N, [&] {
      tools::ExtendedKalmanFilter c(ekf);
      g_sink += size_t(c.x.size());
    });
    std::printf("  ① EKF 拷贝构造                    %8.1f ns  sizeof(EKF)=%zu B\n", t,
                sizeof(tools::ExtendedKalmanFilter));
  }

  // ── ② Target 纯拷贝构造 ──
  auto_aim::Target src(4.0, 1.0, 0.2, 0.1);
  {
    double t = bench(N, [&] {
      auto_aim::Target c(src);
      g_sink += c.armor_xyza_list().size();
    });
    std::printf("  ② Target 纯拷贝构造（3 槽位）      %8.1f ns\n", t);
  }

  // ── ③ 3 次 clone 单独测（用 volatile 防消除）──
  {
    auto w = std::make_unique<auto_aim::EkfStateOnly>();
    auto q = std::make_unique<auto_aim::FixedNoise>();
    auto m = std::make_unique<auto_aim::PassthroughFilter>();
    double t = bench(N, [&] {
      auto a = w->clone();
      auto b = q->clone();
      auto c = m->clone();
      g_sink += size_t(a.get()) + size_t(b.get()) + size_t(c.get());
      g_sink += size_t(a->estimate() * 0);
    });
    std::printf("  ③ 3 次 clone()（全部 stateless 级） %8.1f ns\n", t);
  }

  // ── ④ 空壳对照：只做 3 次 new/delete ──
  {
    struct Tiny { double v; };
    double t = bench(N, [&] {
      auto a = std::make_unique<Tiny>(); auto b = std::make_unique<Tiny>(); auto c = std::make_unique<Tiny>();
      g_sink += size_t(a.get()) + size_t(b.get()) + size_t(c.get());
    });
    std::printf("  ④ 3 次 make_unique<Tiny>（对照）    %8.1f ns  ← 纯堆分配成本\n", t);
  }

  // ── ⑤ 移动构造对照 ──
  {
    double t = bench(N, [&] {
      auto_aim::Target tmp(4.0, 1.0, 0.2, 0.1);
      auto_aim::Target c(std::move(tmp));
      g_sink += c.armor_xyza_list().size();
    });
    std::printf("  ⑤ Target 移动构造（含一次构造）     %8.1f ns\n", t);
  }

  std::printf("\n─ 判读 ────────────────────────────────────────\n");
  std::printf("  · 若 ③ ≈ ④（都很小）而 ② 很大 → **成本不在 unique_ptr，在 Target 的其它成员**\n");
  std::printf("  · 若 ① 占 ② 的大部分 → 真凶是 **EKF（Eigen 矩阵）的拷贝**\n");
  return 0;
}
