/**
 * @file test/function/test_wheels_estimate.cpp
 * @brief ⭐ W43：`utils/wheels/` 里「原插件实现」的单元测试
 *
 * 覆盖从 `archive/target_plugins/` 转出来的 3 组轮子：
 *   ① `estimate/angular_velocity.hpp` —— ω 的 4 种估计
 *   ② `estimate/process_noise.hpp`    —— Q 的 2 种策略（含 B13 方向性修复）
 *   ③ `filter/median_filter.hpp`      —— 量测滤波 2 种
 *
 * ⭐ **验证要点**：轮子**零业务依赖**（只吃 double / Eigen::Vector3d / time_point），
 *    任何人可以在 `test/` 或应用里直接实例化 —— 这就是"优化算法做 utils"的落地方式。
 */
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>

#include "utils/wheels/estimate/angular_velocity.hpp"
#include "utils/wheels/estimate/process_noise.hpp"
#include "utils/wheels/filter/median_filter.hpp"

using namespace utils::wheels;
using TP = std::chrono::steady_clock::time_point;
static int g_fail = 0;
static void ok(const char * m) { std::printf("  [OK] %s\n", m); }
static void bad(const char * m) { std::printf("  [!!] %s\n", m); ++g_fail; }

int main()
{
  std::printf("═══ wheels（原插件实现）单元测试（W43）═══\n");
  const auto t0 = std::chrono::steady_clock::now();
  auto at = [&](double sec) { return t0 + std::chrono::microseconds(int64_t(sec * 1e6)); };

  // ═══ ① 角速度估计 ═══
  std::printf("① 角速度估计（4 种）\n");
  {
    EkfStateOnly e;
    if (e.estimate(3.5) == 3.5) ok("EkfStateOnly 直读 EKF 的 ω（= 同济行为）");
    else bad("EkfStateOnly 错");
  }
  {
    EkfOnly e;
    // 数据不足 → 回退 EKF
    e.update(0.0, at(0.0));
    if (std::fabs(e.estimate(7.0) - 7.0) < 1e-9) ok("EkfOnly 数据不足 → 回退 EKF 值");
    else bad("EkfOnly 回退错");

    // 填满窗口，yaw 每 10 ms 转 0.1 rad → ω ≈ 10 rad/s（被限幅到 10）
    for (int i = 0; i < 5; ++i) e.update(0.1 * i, at(0.01 * i));
    const double w = e.estimate(0.0);
    if (std::fabs(w - 10.0) < 0.5) ok("EkfOnly 首尾中心差分 ≈ 10 rad/s（限幅生效）");
    else { char b[80]; std::snprintf(b, sizeof b, "EkfOnly 差分错: %.3f", w); bad(b); }
  }
  {
    VisualDiff v;
    for (int i = 0; i < 5; ++i) v.update(0.05 * i, at(0.01 * i));   // ω ≈ 5 rad/s
    const double w = v.estimate(0.0);
    if (std::fabs(w - 5.0) < 0.5) ok("VisualDiff 相邻帧加权平均 ≈ 5 rad/s");
    else { char b[80]; std::snprintf(b, sizeof b, "VisualDiff 错: %.3f", w); bad(b); }
  }
  {
    ImuFusion f(0.7);
    for (int i = 0; i < 5; ++i) f.update(0.05 * i, at(0.01 * i));
    const double w = f.estimate(0.0);
    if (std::fabs(w - 5.0) < 0.5) ok("ImuFusion 互补融合 ≈ 5 rad/s（alpha=0.7）");
    else { char b[80]; std::snprintf(b, sizeof b, "ImuFusion 错: %.3f", w); bad(b); }
  }

  // ═══ ② 过程噪声 ═══
  std::printf("② 过程噪声调整（2 种）\n");
  {
    FixedNoise n;
    double v1 = 100, v2 = 400;
    n.adapt(50.0, v1, v2);
    if (v1 == 100 && v2 == 400) ok("FixedNoise 什么都不改（= 同济行为）");
    else bad("FixedNoise 改了值");
  }
  {
    NisBasedAdapter a;
    // ⭐ B13 回归：NIS 大 → Q 必须**变大**（原实现的方向是反的！）
    double v1 = 100, v2 = 400;
    a.adapt(NisBasedAdapter::kNisQ95 * 2.0, v1, v2);
    if (v1 > 100 && v2 > 400) ok("⭐ NIS 大 → v1/v2 **提高**（B13 方向性修复）");
    else { char b[96]; std::snprintf(b, sizeof b, "B13 回归失败: v1=%.1f v2=%.1f", v1, v2); bad(b); }

    // NIS 很小 → 降低
    double v1b = 100, v2b = 400;
    a.adapt(0.01, v1b, v2b);
    if (v1b < 100 && v2b < 400) ok("NIS 很小 → v1/v2 降低");
    else bad("NIS 小 时未降低");

    // 上限保护
    double v1c = 100, v2c = 400;
    for (int i = 0; i < 50; ++i) a.adapt(1000.0, v1c, v2c);
    if (v1c <= a.v1_max() && v2c <= a.v2_max()) ok("连续大 NIS 被上限钳住（不会发散）");
    else bad("上限未生效");
  }

  // ═══ ③ 量测滤波 ═══
  std::printf("③ 量测滤波（2 种）\n");
  {
    PassthroughFilter p;
    const Eigen::Vector3d m(1, 2, 3);
    if ((p.filter(m) - m).norm() < 1e-12) ok("Passthrough 恒等（= 同济行为）");
    else bad("Passthrough 改了值");
  }
  {
    MedianFilter m;
    const Eigen::Vector3d a(1, 1, 1);
    if ((m.filter(a) - a).norm() < 1e-12) ok("MedianFilter 前 2 帧原样返回");
    else bad("MedianFilter 早帧不该滤");

    m.filter(Eigen::Vector3d(1.1, 1.1, 1.1));
    m.filter(Eigen::Vector3d(1.2, 1.2, 1.2));
    const auto out = m.filter(Eigen::Vector3d(100, 100, 100));   // 离群值
    if (out.norm() < 5.0) ok("MedianFilter 抑制离群值（100 → 中位数附近）");
    else { char b[96]; std::snprintf(b, sizeof b, "中值滤波未抑制: %.1f", out.norm()); bad(b); }
  }

  std::printf("\n%s（%s）\n", g_fail == 0 ? "✅ wheels 全部通过" : "❌ 有失败", g_fail == 0 ? "13 项" : "见上");
  return g_fail == 0 ? 0 : 1;
}
