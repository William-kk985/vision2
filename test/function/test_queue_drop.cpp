/// @file test_queue_drop.cpp
/// @brief 队列满导致丢帧的上报机制（`full_handler`）单元测试
///
/// 背景：相机驱动的采集队列 `queue_(1)` 只缓冲 1 帧，模板默认
/// `PopWhenFull = false`，队列满时**丢弃新帧**并回调 `full_handler_`。
/// 在接入本回调之前，丢帧完全静默 —— 主循环只会观察到帧率下降，
/// 无法判断原因是"采集丢帧"还是"消费过慢"。
///
/// 本测试覆盖三件事：
///   ① 队列未满时**不**回调（不能误报）
///   ② 队列满时回调，且**每个被丢弃的帧各回调一次**
///   ③ 丢弃后队列内容保持"最旧那一帧"（不覆盖、不弹出），即行为未被改动
#include "utils/concurrency/thread_safe_queue.hpp"

#include <cstdio>
#include <vector>

static int failures = 0;
#define CHECK(cond, msg)                                            \
  do {                                                              \
    if (!(cond)) { std::printf("  FAIL  %s\n", msg); ++failures; }  \
    else         { std::printf("  ok    %s\n", msg); }              \
  } while (0)

int main()
{
  std::printf("  ==== test_queue_drop ====\n\n");

  // ── ① 未满时不回调 ──
  {
    int calls = 0;
    tools::ThreadSafeQueue<int> q(3, [&] { ++calls; });
    q.push(1);
    q.push(2);
    CHECK(calls == 0, "队列未满 -> full_handler 不触发");

    int v = 0;
    q.pop_for(v, std::chrono::milliseconds(1));
    CHECK(v == 1, "未满时入队的内容按 FIFO 取出");
  }

  // ── ② 满时回调，且每丢一帧回调一次 ──
  {
    int calls = 0;
    tools::ThreadSafeQueue<int> q(2, [&] { ++calls; });
    q.push(10);
    q.push(20);
    CHECK(calls == 0, "恰好填满（size == max_size）-> 仍不回调");

    q.push(30);  // 第 3 个：满 -> 丢弃 + 回调
    CHECK(calls == 1, "第 1 次溢出 -> 回调 1 次");

    q.push(40);
    q.push(50);
    CHECK(calls == 3, "连续溢出 3 帧 -> 回调累计 3 次（每丢一帧一次）");
  }

  // ── ③ 丢弃的是【新】帧，队列内容不被改动（行为未变）──
  {
    int calls = 0;
    tools::ThreadSafeQueue<int> q(2, [&] { ++calls; });
    q.push(1);
    q.push(2);
    q.push(3);  // 丢弃
    q.push(4);  // 丢弃

    int v = 0;
    std::vector<int> got;
    while (q.pop_for(v, std::chrono::milliseconds(1))) got.push_back(v);

    CHECK(got.size() == 2, "容量仍为 2（溢出未扩容）");
    CHECK(got.size() == 2 && got[0] == 1 && got[1] == 2,
          "丢弃的是【新】帧：队列保持最初的 1,2（PopWhenFull=false 的原行为）");
    CHECK(calls == 2, "回调次数 == 被丢弃帧数");
  }

  // ── ④ 未提供 handler 时不应崩溃（默认空回调）──
  {
    tools::ThreadSafeQueue<int> q(1);
    q.push(1);
    q.push(2);  // 满且无 handler
    int v = 0;
    CHECK(q.pop_for(v, std::chrono::milliseconds(1)) && v == 1,
          "无 full_handler 时溢出安全（默认空回调，不崩）");
  }

  std::printf("\n  %s\n", failures == 0 ? "全部通过" : "存在失败");
  return failures == 0 ? 0 : 1;
}
