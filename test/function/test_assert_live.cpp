/**
 * @file test/function/test_assert_live.cpp
 * @brief ⭐⭐⭐ W36：**构建类型自检** —— 保证「assert 真的会触发」
 *
 * ## 为什么需要这个测试
 * 本项目已经**三次**栽在「`assert` 在 Release 是空操作」上。
 * 为此确立了「Debug + Release 双构建跑同一套 ctest」的纪律。
 *
 * ⚠️ 但 W35 我在清 CMakeCache 重配时**漏了 `-DCMAKE_BUILD_TYPE=Debug`**，
 *    导致 `exp/hzmir_dbg` **静默变成 Release**（`CMAKE_BUILD_TYPE` 为空 → 顶层
 *    CMakeLists 回退到 Release）→ **assert 全部失效**，
 *    而 ctest 依旧「17/17 全绿」—— **纪律被绕过却毫无提示**。
 *
 * ## 本测试做什么
 * 1. **核对构建类型**：CMake 把该构建树的**预期类型**通过 `HZMIR_EXPECTED_DEBUG` 传进来，
 *    本测试核对它和 `NDEBUG` 的实际情况是否一致。
 * 2. **真实触发一次 `assert(false)`**（在 `fork()` 出的子进程里）——
 *    子进程**必须异常终止**；若它正常退出，说明 assert 是空操作。
 *
 * ⇒ 这样「Debug 树其实是 Release」会**立刻被测出来**，而不是静默通过。
 */
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <sys/wait.h>
#include <unistd.h>

#ifndef HZMIR_EXPECTED_DEBUG
#  error "缺少 HZMIR_EXPECTED_DEBUG（应由 CMake 传入）"
#endif

int main()
{
  std::printf("═══ 构建类型自检（W36）═══\n");
  std::printf("  CMake 预期本树: %s\n", HZMIR_EXPECTED_DEBUG ? "Debug" : "Release/其它");

  int rc = 0;

  // ── ① 核对 NDEBUG ──
#ifdef NDEBUG
  const bool asserts_live = false;
  std::printf("  NDEBUG:        已定义 → assert 是**空操作**\n");
#else
  const bool asserts_live = true;
  std::printf("  NDEBUG:        未定义 → assert **激活**\n");
#endif

  if (HZMIR_EXPECTED_DEBUG && !asserts_live) {
    std::printf("  [!!] ❌ 本树**预期是 Debug**，但 assert 是空操作！\n");
    std::printf("       ⇒ 请 `rm -f CMakeCache.txt && cmake -DCMAKE_BUILD_TYPE=Debug ...`\n");
    rc = 1;
  } else if (!HZMIR_EXPECTED_DEBUG && asserts_live) {
    std::printf("  [!!] ❌ 本树**预期不是 Debug**，但 assert 却是激活的（构建类型混淆）\n");
    rc = 1;
  } else {
    std::printf("  [OK] 构建类型与 assert 状态**一致**\n");
  }

  // ── ② 真实触发一次 assert(false)，看子进程会不会死 ──
  {
    const pid_t pid = fork();
    if (pid == 0) {
      // 子进程：这句话里的 assert 必须在**激活**时终止进程
      assert(1 == 2 && "这是一个故意的断言失败");
      _exit(0);   // ⭐ 走到这里说明 assert 没生效
    }
    int status = 0;
    waitpid(pid, &status, 0);
    const bool died = !WIFEXITED(status) || WEXITSTATUS(status) != 0;

    if (asserts_live && !died) {
      std::printf("  [!!] ❌ 实测：`assert(false)` **没有**终止进程 → assert 被禁用了\n");
      rc = 1;
    } else if (!asserts_live && died) {
      std::printf("  [!!] ❌ 实测：NDEBUG 已定义但 `assert` 仍终止了进程（矛盾）\n");
      rc = 1;
    } else {
      std::printf("  [OK] 实测：`assert(false)` %s\n",
                  died ? "确实终止了子进程 ✅" : "被正确跳过（Release 预期行为）");
    }
  }

  if (rc == 0)
    std::printf("\n✅ 构建类型自检通过 —— assert 行为可信\n");
  else
    std::printf("\n❌ 构建类型自检失败 —— **这个构建树的 ctest 结果不可信**\n");
  return rc;
}
