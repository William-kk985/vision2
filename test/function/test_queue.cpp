// test/function/test_queue.cpp —— ⭐ W11：并发修复的**量化**验证
//
// 量三件事：
//   ① Target 的一次拷贝要多久、占帧预算多少
//   ② 「旧 API：empty()+front()」 vs 「新 API：try_peek()」 的耗时差
//   ③ push(const T&) vs push(T&&) vs emplace() 的耗时差
#include <chrono>
#include <cstdio>
#include <list>
#include <optional>
#include <vector>

#include "core/types.hpp"
#include "core/auto_aim/target/target.hpp"
#include "utils/concurrency/thread_safe_queue.hpp"

using namespace auto_aim;
using clock_t_ = std::chrono::steady_clock;

static double ns_per(clock_t_::time_point a, clock_t_::time_point b, int n)
{
  return std::chrono::duration_cast<std::chrono::nanoseconds>(b - a).count() / double(n);
}

int main()
{
  const int N = 20000;
  Target t(4.0, 1.0, 0.2, 0.1);
  for (int i = 0; i < 50; ++i) t.predict(0.01);

  std::printf("Target: sizeof=%zu B\n\n", sizeof(Target));

  // ═══ ① Target 的一次拷贝成本 ═══
  {
    auto a = clock_t_::now();
    for (int i = 0; i < N; ++i) { volatile auto copy = t; (void)copy; }
    auto b = clock_t_::now();
    std::printf("① Target 拷贝一次        %8.1f ns   (10ms 帧预算的 %.4f%%)\n",
                ns_per(a, b, N), ns_per(a, b, N) / 1e7 * 100);
  }

  // ═══ ② 旧 API vs 新 API ═══
  {
    tools::ThreadSafeQueue<Target, true> q(1);
    q.push(t);

    // 旧：empty() + front()（两次加锁 + by-value 拷贝 + TOCTOU 风险）
    auto a = clock_t_::now();
    for (int i = 0; i < N; ++i) {
      if (!q.empty()) {
        auto v = q.front();
        (void)v;
      }
    }
    auto b = clock_t_::now();
    const double old_ns = ns_per(a, b, N);

    // 新：try_peek()（一次加锁）
    Target out;
    auto c = clock_t_::now();
    for (int i = 0; i < N; ++i) q.try_peek(out);
    auto d = clock_t_::now();
    const double new_ns = ns_per(c, d, N);

    std::printf("② empty()+front() 旧     %8.1f ns\n", old_ns);
    std::printf("  try_peek()      新     %8.1f ns   → 省 %.1f ns/帧 (%.0f%%)\n",
                new_ns, old_ns - new_ns, (old_ns - new_ns) / old_ns * 100);
  }

  // ═══ ③ push 三种方式 ═══
  {
    tools::ThreadSafeQueue<Target, true> q(1);
    auto a = clock_t_::now();
    for (int i = 0; i < N; ++i) q.push(t);            // const T&  → 队列内再拷一次
    auto b = clock_t_::now();
    Target t2(4.0, 1.0, 0.2, 0.1);
    for (int i = 0; i < 50; ++i) t2.predict(0.01);
    auto c = clock_t_::now();
    for (int i = 0; i < N; ++i) q.push(std::move(t2)); // T&& → 移动
    auto d = clock_t_::now();
    for (int i = 0; i < N; ++i) q.emplace(4.0, 1.0, 0.2, 0.1);  // 原地构造
    auto e = clock_t_::now();

    std::printf("③ push(const T&)         %8.1f ns\n", ns_per(a, b, N));
    std::printf("  push(T&&)              %8.1f ns\n", ns_per(c, d, N));
    std::printf("  emplace()              %8.1f ns\n", ns_per(e == e ? d : d, e, N));
  }

  std::printf("\n✅ 量化完成 —— 注意：这些是**每帧**成本，乘 100 Hz 就是占空比\n");
  return 0;
}
