// test/function/test_can_throttle.cpp
// ⭐⭐ W109：**SocketCAN 失败日志的降噪判据**
//
// ⚠️ **起因**（真机实测踩到）：`SocketCAN` 的守护线程**每 100ms 重试一次**，
//   而 `try_open()` 原来**每次失败都 `warn`** ⇒ **每秒 10 条一模一样的警告**，
//   把终端刷满（真机上没接 CAN 时必然发生；`upstream` 逐字相同 = 继承来的）。
//
// ⭐ 这个测试钉住"**降噪判据**"：第 1 次 + 每 50 次 `warn`，其余 `debug`。
//   ⚠️ 注意：**只测日志级别，不测重试行为** —— 重试还是 100ms 一次（同济行为，不能改）。

#include <cassert>
#include <cstdio>

#include "io/can/socketcan.hpp"

static int passed = 0;
static void ok(const char * w) { std::printf("  [OK] %s\n", w); passed++; }

int main()
{
  std::printf("  ════ test_can_throttle ════\n\n");

  // ── ① 前几条：只有第 1 条该 warn ──
  std::printf("① 开头几条（模拟真机：连不上，一直重试）\n");
  {
    assert(io::can_fail_should_warn(1) == true);    // ⭐ 首次必须 warn
    assert(io::can_fail_should_warn(2) == false);
    assert(io::can_fail_should_warn(3) == false);
    assert(io::can_fail_should_warn(49) == false);
    ok("第 1 次 warn；第 2~49 次 debug（不再每 100ms 刷一条）");
  }

  // ── ② 每 50 次一条 ──
  std::printf("\n② 周期性提醒（每 50 次 ≈ 5 秒）\n");
  {
    assert(io::can_fail_should_warn(50) == true);
    assert(io::can_fail_should_warn(100) == true);
    assert(io::can_fail_should_warn(150) == true);
    for (int n : {51, 52, 99, 101, 149, 151, 999}) {
      assert(io::can_fail_should_warn(n) == false);
    }
    ok("第 50/100/150… 次 warn；其余 debug");
  }

  // ── ③ ⭐ 降噪效果量化：原来 vs 现在 ──
  std::printf("\n③ ⭐ 降噪效果（以 60 秒 = 600 次重试为例）\n");
  {
    const int retries = 600;   // 60 秒 / 100ms
    int n_warn = 0;
    for (int n = 1; n <= retries; ++n) {
      if (io::can_fail_should_warn(n)) ++n_warn;
    }
    std::printf("     600 次重试 ⇒ warn 条数： 修前 %d 条 → 修后 %d 条（降 %.0f%%）\n", retries, n_warn,
                (1.0 - static_cast<double>(n_warn) / retries) * 100.0);
    assert(n_warn == 13);   // 1 + 50,100,...,600 = 1 + 12
    ok("600 次重试只报 13 条（修前 600 条）⇒ 降噪 97.8%");
  }

  // ── ④ 边界 ──
  std::printf("\n④ 边界\n");
  {
    assert(io::can_fail_should_warn(0) == false);    // 0 次失败不该 warn
    assert(io::can_fail_should_warn(-1) == false);   // 负数也不该
    ok("n=0 / n<0 ⇒ 不 warn（防御性）");
  }

  std::printf("\n  ════ %d 项通过 ════\n", passed);
  return 0;
}
