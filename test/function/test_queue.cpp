// test/function/test_queue.cpp —— ⭐ W11：并发修复的**量化**验证
//
// 量三件事：
//   ① Target 的一次拷贝要多久、占帧预算多少
//   ② 「旧 API：empty()+front()」 vs 「新 API：try_peek()」 的耗时差
//   ③ push(const T&) vs push(T&&) vs emplace() 的耗时差
#include <chrono>
#include <thread>
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

  
  // ═══ ⭐⭐ W55：`pop_for` 带超时 —— 防「主循环卡死在 pop() 里导致 Ctrl-C 失效」 ═══
  {
    std::printf("\n══ pop_for（带超时）══\n");
    int fails = 0;

    // ① 空队列 + 短超时 → 必须返回 false（而不是永远等）
    {
      tools::ThreadSafeQueue<int> q(4);
      int v = -1;
      auto a = clock_t_::now();
      const bool got = q.pop_for(v, std::chrono::milliseconds(50));
      auto b = clock_t_::now();
      const double ms = std::chrono::duration<double, std::milli>(b - a).count();
      std::printf("  ① 空队列 pop_for(50ms)  返回 %-5s  实际耗时 %6.1f ms  %s\n",
                  got ? "true" : "false", ms, (!got && ms >= 40 && ms < 300) ? "✅" : "❌");
      if (got || ms < 40 || ms >= 300) ++fails;
    }

    // ② 有数据 → 立即返回 true
    {
      tools::ThreadSafeQueue<int> q(4);
      int v = -1;
      q.push(42);
      const bool got = q.pop_for(v, std::chrono::milliseconds(50));
      std::printf("  ② 有数据 pop_for         返回 %-5s  值=%d  %s\n", got ? "true" : "false", v,
                  (got && v == 42) ? "✅" : "❌");
      if (!got || v != 42) ++fails;
    }

    // ③ ⭐ 真正的场景：另一线程 30ms 后放数据 → 应提前拿到，不是傻等超时
    {
      tools::ThreadSafeQueue<int> q(4);
      std::thread feeder([&q] {
        std::this_thread::sleep_for(std::chrono::milliseconds(30));
        q.push(7);
      });
      int v = -1;
      auto a = clock_t_::now();
      const bool got = q.pop_for(v, std::chrono::milliseconds(500));
      const double ms = std::chrono::duration<double, std::milli>(clock_t_::now() - a).count();
      std::printf("  ③ 另一线程 30ms 后放数据 返回 %-5s  值=%d  耗时 %6.1f ms  %s\n",
                  got ? "true" : "false", v, ms, (got && v == 7 && ms < 200) ? "✅" : "❌");
      if (!got || v != 7 || ms >= 200) ++fails;
      feeder.join();
    }

    std::printf("  %s\n", fails == 0 ? "✅ pop_for 三项全过（不会无限阻塞）" : "❌ 有失败");
  }

  std::printf("\n✅ 量化完成 —— 注意：这些是**每帧**成本，乘 100 Hz 就是占空比\n");
  return 0;
}
