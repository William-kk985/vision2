// test/function/test_chi2.cpp —— ⭐ W24：χ² 分位表验证
#include <cassert>
#include <cstdio>

#include "utils/math/chi2.hpp"

static int passed = 0;
static void ok(const char * w) { std::printf("  [OK] %s\n", w); passed++; }

int main()
{
  std::printf("① dof=4（EKF 观测维度）的标准值\n");
  {
    const auto q = tools::chi2_quantiles(4);
    std::printf("     mean=%.4f  q05=%.5f  q50=%.4f  q95=%.4f  q99=%.4f\n", q.mean, q.q05, q.q50,
                q.q95, q.q99);
    assert(std::abs(q.mean - 4.0) < 1e-9);
    assert(std::abs(q.q05 - 0.71072) < 1e-5);
    assert(std::abs(q.q50 - 3.3567) < 1e-4);
    assert(std::abs(q.q95 - 9.4877) < 1e-4);
    assert(std::abs(q.q99 - 13.2767) < 1e-4);
    ok("dof=4: mean=4, q05=0.7107, q95=9.4877, q99=13.2767");
  }

  std::printf("② ⭐ 复现 W9 的关键发现\n");
  {
    // EKF 原代码用 0.711 当"95% 置信"阈值
    const double used = 0.711;
    const double correct = tools::chi2_q95(4);
    const double tail = tools::chi2_q05(4);
    std::printf("     原代码用 %.3f；χ²(4) 的 5%% 下尾 = %.5f，95%% 分位 = %.4f\n", used, tail,
                correct);
    assert(std::abs(used - tail) < 1e-2);          // 它其实是下尾
    assert(std::abs(used - correct) > 8.0);        // 与 95% 分位差 13 倍
    std::printf("     → 原阈值其实是【下尾】，与 95%% 分位差 %.1f 倍\n", correct / used);
    ok("确认 `0.711` = χ²(4) 的 **5% 下尾**，不是 95% 分位（差 13.3 倍）");
  }

  std::printf("③ 各自由度单调性\n");
  {
    for (int dof = 1; dof <= 10; ++dof) {
      const auto q = tools::chi2_quantiles(dof);
      assert(q.q05 < q.q50 && q.q50 < q.q95 && q.q95 < q.q99);
      assert(std::abs(q.mean - dof) < 1e-9);
    }
    ok("dof 1~10 全部满足 q05 < q50 < q95 < q99 且 mean=dof");
  }

  std::printf("④ 大自由度走 Wilson–Hilferty 近似\n");
  {
    const auto q = tools::chi2_quantiles(20);
    std::printf("     dof=20: mean=%.1f q95=%.3f q99=%.3f（真值 31.410 / 37.566）\n", q.mean, q.q95,
                q.q99);
    assert(std::abs(q.q95 - 31.410) < 0.5);
    assert(std::abs(q.q99 - 37.566) < 0.8);
    ok("dof>10 用近似，误差 < 0.5（够用）");
  }

  std::printf("⑤ 边界\n");
  {
    assert(tools::chi2_q95(0) == tools::chi2_q95(1));   // dof<=0 视为 1
    assert(tools::chi2_mean(0) == 1.0);
    ok("dof<=0 安全回退到 1");
  }

  std::printf("\n✅ χ² 分位表验证通过（%d 项）\n", passed);
  return 0;
}
