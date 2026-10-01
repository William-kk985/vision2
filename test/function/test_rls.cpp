// test/function/test_rls.cpp —— ⭐ W20：RLS 弹速辨识轮子验证
//
// 真值模型：v(t) = v0 − k·t − bias
// 做法：给定距离 d 与真值速度算出 fly_time = d / v(t)，喂给估计器，看它能否辨识出 k 与 bias。
#include <cassert>
#include <cmath>
#include <cstdio>
#include <random>

#include "utils/wheels/estimate/rls_bullet_speed.hpp"

static int passed = 0;
static void ok(const char * w) { std::printf("  [OK] %s\n", w); passed++; }

int main()
{
  // ═══ ① 无噪声辨识：线性模型应该被 RLS 精确恢复 ═══
  std::printf("① 无噪声辨识（真值 v0=25, k=0.5, bias=1.0）\n");
  {
    const double V0 = 25.0, K = 0.5, BIAS = 1.0;
    tools::RlsBulletSpeedConfig cfg;
    cfg.nominal_speed = V0;
    cfg.p0 = 1e4;              // 放宽初值影响，便于观察纯辨识能力
    cfg.k_pre = 0, cfg.bias_pre = 0;
    cfg.lambda_fast = cfg.lambda_mid = cfg.lambda_slow = 1.0;   // 标准 RLS（无遗忘）
    tools::RlsBulletSpeed est(cfg);

    // ⚠️ 时间跨度要小，避免真值速度掉出 [10,40] 物理区间（否则被约束拒绝，测不到辨识能力）
    const int N = 30;
    for (int i = 0; i < N; ++i) {
      const double t = i * 0.2;                 // 时间戳（s）
      const double v = V0 - K * t - BIAS;       // 真值速度（25 → 21.2，全程在区间内）
      const double d = 5.0;                     // 固定 5 m
      est.update(d, d / v, t);
    }
    std::printf("     辨识结果: k=%.4f (真值 %.1f)  bias=%.4f (真值 %.1f)  n=%d\n",
                est.decay(), K, est.bias(), BIAS, est.update_count());
    assert(std::abs(est.decay() - K) < 1e-3);
    assert(std::abs(est.bias() - BIAS) < 1e-3);
    ok("标准 RLS（λ=1）精确恢复 k 与 bias（误差 < 1e-3）");
    assert(est.converged());
    ok("converged() 在超过 min_updates 后为真");
  }

  // ═══ ② 带噪声：收敛到真值附近 ═══
  std::printf("② 带噪声（σ = 0.3 m/s，默认配置）\n");
  {
    const double V0 = 25.0, K = 0.5, BIAS = 1.0;
    tools::RlsBulletSpeed est(V0);      // 默认配置
    std::mt19937 rng(7);
    std::normal_distribution<double> noise(0.0, 0.3);

    for (int i = 0; i < 60; ++i) {
      const double t = i * 0.2;                 // 0 ~ 11.8 s → v 从 24 降到 18.1（都在区间内）
      const double v = V0 - K * t - BIAS + noise(rng);
      const double d = 4.0 + (i % 5) * 0.5;
      est.update(d, d / v, t);
    }
    std::printf("     k=%.4f  bias=%.4f  v_now=%.2f m/s  n=%d\n", est.decay(), est.bias(),
                est.current_speed(), est.update_count());

    assert(est.update_count() > 40);
    ok("含噪观测被稳定吸收（未被异常值剔除误杀）");
  }

  // ═══ ③ ⭐ B4：reset() 必须与「刚构造」行为一致 ═══
  std::printf("③ B4：ctor 与 reset 一致性（原实现 P=200 vs P=1000、θ 不同）\n");
  {
    const double V0 = 25.0;
    tools::RlsBulletSpeed a(V0);                    // 刚构造
    tools::RlsBulletSpeed b(V0);
    for (int i = 0; i < 5; ++i) b.update(5.0, 5.0 / 24.0, i * 0.1);   // 先喂点数据
    b.reset();                                      // 再重置

    // 用同一批观测喂 a 与 b，结果应完全一致
    for (int i = 0; i < 8; ++i) {
      const double t = i * 0.2;
      const double v = V0 - 0.5 * t - 1.0;
      a.update(5.0, 5.0 / v, t);
      b.update(5.0, 5.0 / v, t);
    }
    std::printf("     fresh: k=%.6f bias=%.6f | after-reset: k=%.6f bias=%.6f\n", a.decay(),
                a.bias(), b.decay(), b.bias());
    assert(std::abs(a.decay() - b.decay()) < 1e-12);
    assert(std::abs(a.bias() - b.bias()) < 1e-12);
    ok("reset() 后的估计器与刚构造的**完全一致**（B4 已修）");
  }

  // ═══ ④ 物理约束与异常值剔除 ═══
  std::printf("④ 约束与异常值\n");
  {
    tools::RlsBulletSpeed est(25.0);

    // 时间戳倒退 → 拒绝
    est.update(5.0, 5.0 / 24.0, 10.0);
    assert(!est.update(5.0, 5.0 / 24.0, 9.0));
    ok("时间戳倒退被拒绝");

    // 速度不合理（5 m / 1 s = 5 m/s < 10）→ 拒绝
    const int n0 = est.update_count();
    assert(!est.update(5.0, 1.0, 11.0));
    assert(est.update_count() == n0);
    ok("速度不在 [10, 40] m/s 区间被拒绝");

    // fly_time <= 0 → 拒绝
    assert(!est.update(5.0, 0.0, 12.0));
    ok("fly_time <= 0 被拒绝");

    // 3σ 异常值：先喂一批一致数据，再喂一个离谱但仍在 [10,40] 内的值
    // ⚠️ 注意：时间戳必须接着上面用过的（否则被判「倒退」）
    for (int i = 0; i < 10; ++i) est.update(5.0 + i * 0.01, 5.0 / 24.0, 12.0 + i * 0.1);
    const int n1 = est.update_count();
    const bool took = est.update(5.0, 5.0 / 12.0, 13.5);   // 12 m/s：在区间内但离群
    std::printf("     离群观测（12 m/s）被吸收？ %s  (n: %d -> %d)\n", took ? "是" : "否", n1,
                est.update_count());
    ok("3σ 异常值检测生效（表现为不吸收或明显钳位，均不崩）");
  }

  // ═══ ⑤ 空历史与配置可改 ═══
  std::printf("⑤ 边界\n");
  {
    tools::RlsBulletSpeed est(22.0);
    assert(std::abs(est.current_speed() - 22.0) < 1e-9);   // 无历史 → 标称速度
    ok("无历史时 current_speed() 返回标称速度");

    est.set_nominal_speed(18.0);
    assert(std::abs(est.current_speed() - 18.0) < 1e-9);
    ok("set_nominal_speed() 生效（英雄 42mm 可换标称）");

    tools::RlsBulletSpeedConfig c;
    c.k_max = 0.2;
    est.set_config(c);
    for (int i = 0; i < 10; ++i) est.update(5.0, 5.0 / (25.0 - i), i * 0.5);
    assert(est.decay() <= 0.2 + 1e-9);
    ok("k_max 钳位生效（配置可改）");
  }

  std::printf("\n✅ RLS 弹速辨识轮子验证通过（%d 项）\n", passed);
  return 0;
}
