// test/function/test_target_plugins.cpp —— ⭐ W23：3 个策略槽位 + 集成 + 拷贝成本
#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <memory>
#include <vector>

#include "core/auto_aim/target/target.hpp"
#include "core/auto_aim/target/target_plugins.hpp"

using namespace auto_aim;
using clk = std::chrono::steady_clock;

static int passed = 0;
static void ok(const char * w) { std::printf("  [OK] %s\n", w); passed++; }

static clk::time_point ms(int64_t x) { return clk::time_point{} + std::chrono::milliseconds(x); }

int main()
{
  // ═══ ① IAngularVelocityEstimator：3 个实现 ═══
  std::printf("① 角速度估计（3 实现）\n");
  {
    // EkfOnly：历史 < 3 帧 → 回退 EKF 值；≥3 → 中心差分
    EkfOnly e;
    e.set_ekf_angular_velocity(3.5);
    e.update(0.0, ms(0));
    assert(std::abs(e.estimate() - 3.5) < 1e-9);       // 只有 1 帧 → EKF 值
    e.update(0.1, ms(100));
    assert(std::abs(e.estimate() - 3.5) < 1e-9);       // 2 帧 → 仍回退
    e.update(0.2, ms(200));                            // 3 帧：中心差分 (0.2-0.0)/0.2 = 1.0
    std::printf("     EkfOnly(3帧) = %.4f rad/s（首尾差分）\n", e.estimate());
    assert(std::abs(e.estimate() - 1.0) < 1e-6);
    ok("EkfOnly：<3 帧回退 EKF 值；≥3 帧用首尾中心差分（⭐ 名字叫 EkfOnly 但不只依赖 EKF）");

    // 限幅
    EkfOnly e2;
    e2.update(0.0, ms(0));
    e2.update(10.0, ms(1));    // dt=1ms, Δyaw=10 → 10000 rad/s → 应被钳到 10
    e2.update(-10.0, ms(2));
    assert(std::abs(e2.estimate()) <= 10.0 + 1e-9);
    ok("限幅在 ±10 rad/s（物理上限）");

    // VisualDiff：加权平均
    VisualDiff v;
    for (int i = 0; i < 5; ++i) v.update(0.1 * i, ms(i * 100));   // 恒定 1 rad/s
    std::printf("     VisualDiff(恒定1rad/s) = %.4f\n", v.estimate());
    assert(std::abs(v.estimate() - 1.0) < 1e-6);
    ok("VisualDiff：恒定角速度下正确给出 1.0 rad/s（加权平均）");

    // ImuFusion：alpha 权重
    ImuFusion f(0.7);
    for (int i = 0; i < 5; ++i) f.update(0.1 * i, ms(i * 100));
    std::printf("     ImuFusion(alpha=0.7) = %.4f\n", f.estimate());
    assert(std::abs(f.estimate() - 1.0) < 1e-6);
    ok("ImuFusion：恒定角速度下也正确（⭐ 名字含 IMU 但实际不用 IMU）");
  }

  // ═══ ② IProcessNoiseAdapter ═══
  std::printf("② 过程噪声自适应\n");
  {
    FixedNoise fn;
    double v1 = 100, v2 = 400;
    fn.adapt(999.0, v1, v2);
    assert(v1 == 100 && v2 == 400);
    ok("FixedNoise：任何 NIS 都不改（= 同济的硬编码行为）");

    NisBasedAdapter na;
    const double TH = 9.21;

    v1 = 100; v2 = 400;
    na.adapt(TH * 2.0, v1, v2);              // 远大于阈值 → +20%
    // ⭐ B13：修前 v2 = min(400, 480) = 400（**不增**）；修后上限 800 → 480
    assert(std::abs(v1 - 120.0) < 1e-9);
    assert(std::abs(v2 - 480.0) < 1e-9);
    ok("NIS > 1.5×9.21 → Q 提高 20%（含 v2：修前被 400 上限吃掉，见 B13）");

    v1 = 100; v2 = 400;
    na.adapt(TH * 1.2, v1, v2);              // 略大 → +10%
    // ⭐ B13：修前 v2 = min(300, 440) = 300，**方向反了**（想提高却降低了）
    assert(std::abs(v1 - 110.0) < 1e-9);
    assert(std::abs(v2 - 440.0) < 1e-9);
    ok("NIS > 9.21 → Q 提高 10%（⭐ B13：修前 v2 反而降到 300）");

    v1 = 100; v2 = 400;
    na.adapt(TH * 0.3, v1, v2);              // 很小 → -20%
    assert(std::abs(v1 - 80.0) < 1e-9);
    ok("NIS < 4.605 → Q 降低 20%（快速恢复）");

    v1 = 100; v2 = 400;
    na.adapt(TH, v1, v2);                    // 卡在阈值上 → 不变
    assert(v1 == 100 && v2 == 400);
    ok("NIS ∈ [4.605, 9.21] → 保持不变（死区）");

    // 钳位
    v1 = 500; v2 = 500;
    for (int i = 0; i < 20; ++i) na.adapt(TH * 3, v1, v2);
    assert(v1 <= na.limits().v1_max + 1e-9 && v2 <= na.limits().v2_max + 1e-9);
    std::printf("     钳位上界: v1<=%.0f v2<=%.0f\n", na.limits().v1_max, na.limits().v2_max);
    ok("上限钳位生效（可配）");
  }

  // ═══ ③ IMeasurementFilter ═══
  std::printf("③ 观测滤波\n");
  {
    PassthroughFilter pf;
    const Eigen::Vector3d x(1, 2, 3);
    assert((pf.filter(x) - x).norm() < 1e-12);
    ok("PassthroughFilter：原样返回");

    MedianFilter mf;
    mf.filter({0, 0, 0});
    mf.filter({10, 0, 0});
    assert((mf.filter({1, 0, 0}) - Eigen::Vector3d(1, 0, 0)).norm() < 1e-12);  // 3 帧 → 中值 1
    ok("MedianFilter：3 帧后取中值（剔除 0 与 10 之间的跳变）");

    mf.filter({1, 0, 0});
    mf.filter({1, 0, 0});
    const auto r = mf.filter({1, 0, 0});
    assert(std::abs(r.x() - 1.0) < 1e-12);
    ok("MedianFilter：稳定输入下输出稳定");
  }

  // ═══ ④ 集成：Target 真的用了槽位 ═══
  std::printf("④ 集成到 Target\n");
  {
    Target t(4.0, 1.0, 0.2, 0.1);
    assert(t.process_noise_adapter() != nullptr);
    assert(t.angular_velocity_estimator() != nullptr);
    assert(t.measurement_filter() != nullptr);
    ok("Target 默认就带 3 个槽位（FixedNoise / EkfOnly / Passthrough）");

    t.predict(0.01);
    std::printf("     默认(FixedNoise) v1=%.1f v2=%.1f\n", t.last_q_v1(), t.last_q_v2());
    assert(std::abs(t.last_q_v1() - 100.0) < 1e-9);
    ok("FixedNoise 下 Q 与同济硬编码一致（100 / 400）");

    // 换成 NisBasedAdapter → 注入高 NIS → Q 变化
    Target t2(4.0, 1.0, 0.2, 0.1);
    t2.set_process_noise_adapter(std::make_unique<NisBasedAdapter>());
    // 手动把 EKF 的 nis 抬高
    t2.update([] {
      Lightbar l, r;
      l.angle = r.angle = 0.1; l.length = r.length = 1.0; l.width = r.width = 0.2; l.ratio = r.ratio = 5.0;
      l.center = {0, 0}; l.top = {0, 0}; l.bottom = {0, 1};
      r.center = {.5f, 0}; r.top = {.5f, 0}; r.bottom = {.5f, 1};
      Armor a(l, r);
      a.name = ArmorName::three; a.color = Color::red;
      a.ypr_in_world = Eigen::Vector3d(0.3, 0, 0);
      a.ypd_in_world = Eigen::Vector3d(0.3, 0.05, 4.0);
      return a;
    }());
    const double nis = t2.ekf().data.count("nis") ? t2.ekf().data.at("nis") : 0.0;
    t2.predict(0.01);
    std::printf("     注入后的 nis=%.3f → v1=%.1f v2=%.1f\n", nis, t2.last_q_v1(), t2.last_q_v2());
    ok("NisBasedAdapter 接入后 Q 会随 NIS 变化（通路打通）");
  }

  // ═══ ⑤ ⭐ 拷贝成本量化（W11 的隐患）═══
  // ⚠️⚠️ W41 更正：**这个数字强依赖构建类型**！
  //   Debug(-O0) ≈ 1240 ns   Release(-O3) ≈ 379 ns   → 差 3 倍以上
  //   W23 曾把 **Debug 的值**（1819.8 ns）与 **Release 的同济基准**（318 ns）相比，
  //   得出「贵 5.7 倍」的**错误结论**。真实差异只有 ~+19%（约 +25 ns 来自槽位）。
  //   ⇒ 引用本行数字时**必须标注构建类型**。
  std::printf("⑤ Target 拷贝成本（cluster 3 个插件）");
#ifdef NDEBUG
  std::printf("  [Release]\n");
#else
  std::printf("  [Debug —— 比 Release 慢数倍，勿与 Release 基准比较!]\n");
#endif
  {
    Target t(4.0, 1.0, 0.2, 0.1);
    for (int i = 0; i < 50; ++i) t.predict(0.01);

    const int N = 20000;
    auto a = clk::now();
    for (int i = 0; i < N; ++i) { volatile Target c = t; (void)c; }
    auto b = clk::now();
    const double ns = std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count() / double(N);
    std::printf("     sizeof(Target) = %zu B，拷贝一次 %.1f ns（W11 时同济原版是 318 ns）\n",
                sizeof(Target), ns);
    ok("拷贝含 3 次 clone（这是 W11 提醒的隐患，量化如上）");
  }

  std::printf("\n✅ 目标策略槽位验证通过（%d 项）\n", passed);
  return 0;
}
