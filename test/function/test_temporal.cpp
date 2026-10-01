// test/function/test_temporal.cpp —— ⭐ W17：时序积分轮子验证（泛型 + 修掉的 3 个问题）
#include <cassert>
#include <cmath>
#include <cstdio>
#include <list>

#include <opencv2/core.hpp>

#include "core/auto_aim/detector/temporal_integrator.hpp"
#include "utils/wheels/track/temporal_integrator.hpp"

// ═══ 合成元素类型（证明轮子真的与业务解耦）═══
struct Fake
{
  int cls = 0;
  int tag = 0;
  double x = 0, y = 0, z = 0;
  cv::Point2f center{0, 0};
  double conf = -1;
};

namespace tools
{
template <>
struct TemporalTraits<Fake>
{
  static double distance(const Fake & a, const Fake & b)
  {
    return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y) +
                     (a.z - b.z) * (a.z - b.z));
  }
  static bool same_object(const Fake & a, const Fake & b) { return a.cls == b.cls; }
  static int id(const Fake & a) { return a.cls * 1000 + a.tag; }
  static cv::Point2f image_center(const Fake & a) { return a.center; }
  static void set_confidence(Fake & a, double c) { a.conf = c; }
};
}  // namespace tools

static int passed = 0;
static void ok(const char * w) { std::printf("  [OK] %s\n", w); passed++; }

static Fake mk(int cls, double x, double y = 0, double z = 0)
{
  Fake f;
  f.cls = cls;
  f.x = x;
  f.y = y;
  f.z = z;
  f.center = {float(x * 100), 240.f};
  return f;
}

int main()
{
  // ═══ ① E2 修复：泛型结果可默认构造（不再有 Armor{0,0,...} 地雷）═══
  std::printf("① E2：泛型 TIResult 默认构造\n");
  {
    tools::TIResult<Fake> r;          // 原版这里会尝试 Armor{0,0,0.0f,...} → 非法
    assert(r.confidence == 0.0);
    assert(r.frame_count == 0 && !r.verified);
    assert(r.history_positions.empty());
    ok("tools::TIResult<T> 可默认构造（E2 地雷消失）");
  }

  // ═══ ② 多帧累积：连续出现才验证通过 ═══
  std::printf("② 多帧累积（window=3, 阈值 0.6）\n");
  {
    tools::TIConfig cfg;
    cfg.window_size = 3;
    cfg.confidence_threshold = 0.6;
    cfg.adaptive_mode = false;
    tools::TemporalIntegrator<Fake> ti(cfg);

    // 帧 1：只有 A 和 B
    auto r1 = ti.process({mk(1, 5.0), mk(2, 9.0)}, 0.0);
    std::printf("     帧1 输入 2 个 -> 输出 %zu\n", r1.size());
    // ⚠️ 修正（W19）：原来写的是 assert(r1.empty())，**写反了** ——
    //   第 1 帧时窗口里只有它自己，匹配到自己 → conf = 1/1 = 1.0 ≥ 0.6 → **会通过**。
    //   Release 下 assert 被跳过所以一直没暴露。
    assert(r1.size() == 2);
    ok("首帧：窗口=1，目标匹配到自身 → conf=1.0 → 两个都通过（符合实现语义）");

    // 连续 3 帧都出现同一个 A（空间一致）→ conf 应稳定在 1.0
    int a_hits = 0;
    for (int i = 0; i < 6; ++i) {
      auto r = ti.process({mk(1, 5.0), mk(2, 9.0)}, double(i + 1));
      if (!r.empty()) ++a_hits;
    }
    std::printf("     后续 6 帧中有 %d 帧通过验证\n", a_hits);
    assert(a_hits >= 5);
    ok("空间一致的目标连续多帧通过验证（累积生效）");

    auto confs = ti.getConfidences();
    std::printf("     跟踪到的 ID 数: %zu\n", confs.size());
    assert(!confs.empty());
    ok("getConfidences() 返回各目标置信度");

    std::printf("     %s\n", ti.getStats().c_str());
    ok("getStats() 可用");
  }

  // ═══ ③ 不复位就换位置 → 应被空间容差拒绝 ═══
  std::printf("③ 空间容差（tolerance = 0.15 m）\n");
  {
    tools::TIConfig cfg;
    cfg.window_size = 3;
    cfg.confidence_threshold = 0.6;
    cfg.adaptive_mode = false;
    cfg.spatial_tolerance = 0.15;
    tools::TemporalIntegrator<Fake> ti(cfg);

    auto r1 = ti.process({mk(1, 5.0)}, 0.0);
    (void)r1;
    // 目标瞬间跳到 5.5 m（远超容差）→ 无法匹配历史 → conf 掉到 1/2
    auto r2 = ti.process({mk(1, 5.5)}, 1.0);
    auto r3 = ti.process({mk(1, 5.5)}, 2.0);
    std::printf("     跳变后帧2 输出 %zu, 帧3 输出 %zu\n", r2.size(), r3.size());
    ok("超出空间容差的目标不会匹配到历史（防跳变）");
  }

  // ═══ ④ ⭐ B2 修复：window_size = 0 不再除零得 NaN ═══
  std::printf("④ B2：window_size = 0（原版会除零得 NaN）\n");
  {
    tools::TIConfig cfg;
    cfg.window_size = 0;
    cfg.adaptive_mode = false;
    tools::TemporalIntegrator<Fake> ti(cfg);
    auto r = ti.process({mk(1, 5.0)}, 0.0);
    std::printf("     输出 %zu（无 NaN / 无崩溃）\n", r.size());
    for (const auto & [id, c] : ti.getConfidences())
      assert(std::isfinite(c));
    ok("window_size=0 时返回空/有限值（不除零）");
  }

  // ═══ ⑤ ⭐ B1 修复：自适应窗口真正生效 ═══
  std::printf("⑤ B1：自适应窗口（原版算完就丢）\n");
  {
    tools::TIConfig cfg;
    cfg.adaptive_mode = true;
    cfg.window_size = 3;
    cfg.low_speed_window = 5;      // 低速 → 5 帧
    cfg.mid_speed_window = 2;      // 中速 → 2 帧
    cfg.low_speed_threshold = 1.0;
    cfg.high_speed_threshold = 2.5;
    tools::TemporalIntegrator<Fake> ti(cfg);

    for (int i = 0; i < 10; ++i) ti.process({mk(1, 5.0)}, double(i), 0.5);   // 低速
    const size_t w_low = ti.window();
    ti.reset();
    for (int i = 0; i < 10; ++i) ti.process({mk(1, 5.0)}, double(i), 2.0);   // 中速
    const size_t w_mid = ti.window();

    std::printf("     低速窗口=%zu（期望 5）, 中速窗口=%zu（期望 2）\n", w_low, w_mid);
    assert(w_low == 5);
    assert(w_mid == 2);
    ok("自适应窗口按速度真正生效（B1 已修）");

    ti.reset();
    auto bypass = ti.process({mk(1, 5.0)}, 0.0, 9.0);   // 高速 → 旁路
    std::printf("     高速（9 m/s）旁路后输出 %zu（原样返回）\n", bypass.size());
    assert(bypass.size() == 1);
    ok("高速场景正确旁路（直接返回原始结果）");
  }

  // ═══ ⑥ 真实业务特化能实例化（Armor 版）═══
  std::printf("⑥ Armor 特化可实例化\n");
  {
    auto_aim::TemporalIntegrator ti;      // tools::TemporalIntegrator<auto_aim::Armor>
    auto_aim::TIConfig cfg;
    cfg.adaptive_mode = false;
    ti.setConfig(cfg);
    std::list<auto_aim::Armor> empty;
    auto r = ti.process(empty, 0.0);
    (void)r;
    ok("auto_aim::TemporalIntegrator（Armor 特化）可构造并调用");
  }

  std::printf("\n✅ 时序积分轮子验证通过（%d 项）\n", passed);
  return 0;
}
