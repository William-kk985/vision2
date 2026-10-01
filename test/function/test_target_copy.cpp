/**
 * @file test/function/test_target_copy.cpp
 * @brief ⭐⭐ W43：`Target` 的**体积/拷贝成本回归测试**
 *
 * ## 作用
 * `Target` 在**每帧热路径**上（`Planner::plan(Target)` 按值传），
 * 所以**任何让它变胖的改动都该被测出来**。本测试把当前基线钉死：
 *
 * | 基线（同济 `main` 原版 + E2 默认值） | 值 |
 * |---|---|
 * | `sizeof(Target)` | **304 B** |
 * | 拷贝一次（Release） | **~290 ns** |
 *
 * ⚠️ **历史教训**（本项目栽过）：
 * - W23 给 `Target` 加了 3 个 `unique_ptr` 策略槽位 → `sizeof` **344 B**、拷贝 **+9~14%**
 *   → **W43 已移除**（改为 `utils/wheels/` 的独立轮子），见 `archive/target_plugins/FROZEN.md`
 * - 当时还**两次报错数字**：先说「贵 5.7×」（Debug/Release 混用），
 *   后说「反而更快」（测量环境偏差）→ ⭐ **性能数字必须同机交替测、并标注构建类型**
 *
 * ## 阈值
 * `sizeof` 卡 **320 B**（同济 304 + 余量）；**一旦超了本测试就会失败**，
 * 逼你回答「这个新增的成员值不值」。
 */
#include <chrono>
#include <cstdio>

#include "core/auto_aim/target/target.hpp"

using Clock = std::chrono::steady_clock;
static volatile size_t g_sink = 0;

int main()
{
  constexpr int N = 200000;
  std::printf("═══ Target 体积/拷贝回归（W43）═══\n");

  const size_t sz = sizeof(auto_aim::Target);
  std::printf("  sizeof(Target) = %zu B   （同济原版 304 B）\n", sz);
#ifdef NDEBUG
  std::printf("  构建类型: Release\n");
#else
  std::printf("  构建类型: Debug（拷贝数字会慢数倍，勿与 Release 比）\n");
#endif

  // ⭐ 阈值守卫：不许悄悄变胖
  if (sz > 320) {
    std::printf("  [!!] ❌ sizeof 超过 320 B 阈值 → 有人给 Target 加了成员，请确认是否值得\n");
    return 1;
  }
  std::printf("  [OK] sizeof 在阈值内（<= 320 B）\n");

  auto_aim::Target t(4.0, 1.0, 0.2, 0.1);
  for (int i = 0; i < 1000; ++i) { auto c = t; g_sink += c.armor_xyza_list().size(); }

  double best = 1e18, sum = 0;
  for (int r = 0; r < 5; ++r) {
    auto a = Clock::now();
    for (int i = 0; i < N; ++i) { auto c = t; g_sink += c.armor_xyza_list().size(); }
    auto b = Clock::now();
    const double ns = std::chrono::duration<double, std::nano>(b - a).count() / N;
    if (ns < best) best = ns;
    sum += ns;
  }
  const double mean = sum / 5;
  std::printf("  拷贝一次: best=%.1f ns  mean=%.1f ns  （同济 Release 基线 ~290 ns）\n", best, mean);
  std::printf("  占 10 ms 帧预算: %.4f%%\n", mean / 1e7 * 100);

#ifdef NDEBUG
  // ⭐ 阈值**只在 Release 生效** —— Debug 慢 3~5 倍是正常的（本项目栽过"混用构建类型"的坑）
  if (best > 600.0) {
    std::printf("  [!!] ❌ Release 下拷贝异常慢（> 600 ns，基线 ~290 ns）→ 有东西变重了\n");
    return 1;
  }
  std::printf("  [OK] 拷贝成本正常（Release 阈值 600 ns）\n");
#else
  std::printf("  [--] Debug 下不卡时间阈值（只卡 sizeof）—— 这是刻意的\n");
#endif

  std::printf("\n✅ Target 体积/拷贝回归通过\n");
  return 0;
}
