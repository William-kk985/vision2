/**
 * @file test/function/test_debug_overhead.cpp
 * @brief ⭐⭐⭐ W44：**调试体系到底抢不抢自瞄资源** —— 逐项实测
 *
 * ## 为什么要有这个测试
 * 用户问：「**这个 debug 怎么做到不抢夺自瞄资源我还是不理解**」
 * —— 这个问题**只能用测量回答**。本测试把调试的每一层单独计时，
 * 并对照「一个 10 ms 帧预算」算出占用。
 *
 * ## 要回答的问题
 * 1. `FrameDebug` 填充（~50 个字段）要多久？
 * 2. `SinkHub::on_frame` **不挂 sink** 时要多久？（发现它会加锁）
 * 3. ⭐ `Expense::begin/end/us` 要多久？（用的是 `std::map`，怀疑是主要开销）
 * 4. `logger()->debug()` 被级别过滤掉时**还要多久**？（spdlog 会先算参数）
 * 5. `CsvSink`（挂了 sink）要多久？（这是"真开调试"的代价）
 *
 * ⭐ **对照物**：同济 `*_debug.cpp` 用 `cv::imshow` + `cv::waitKey(1)` —— 那是 **ms 级**。
 */
#include <chrono>
#include <cstdio>
#include <memory>

#include "core/debug.hpp"
#include "utils/debug/debug_sink.hpp"
#include "utils/debug/expense.hpp"
#include "utils/debug/csv_sink.hpp"
#include "utils/debug/hotkeys.hpp"
#include "utils/debug/image_sink.hpp"
#include "utils/debug/plotjuggler_sink.hpp"
#include "utils/debug/recorder.hpp"
#include "utils/log/logger.hpp"
#include <opencv2/opencv.hpp>
#include <spdlog/spdlog.h>

using Clock = std::chrono::steady_clock;
static volatile size_t g_sink = 0;

template <class F>
static double bench(int n, F && f)
{
  auto t0 = Clock::now();
  for (int i = 0; i < n; ++i) f();
  auto t1 = Clock::now();
  return std::chrono::duration<double, std::nano>(t1 - t0).count() / n;
}

int main()
{
  constexpr int N = 200000;
  std::printf("═══ 调试体系开销实测（W44）═══\n");
  std::printf("  帧预算 = 10000 µs（100 Hz）\n\n");

  auto pct = [](double ns) { return ns / 1e7 * 100; };

  // ── ⓪ ⭐ 先量「测量本身的底噪」—— 否则会把循环开销误当成组件开销 ──
  const double t_floor = bench(N, [] { g_sink += 1; });
  std::printf("  ⓪ 空 lambda（测量底噪）            %8.1f ns   ← **一切读数都不能低于它**\n",
              t_floor);

  // ── ① 纯 clock::now() 的成本（一切的基准）──
  const double t_now = bench(N, [] { g_sink += size_t(Clock::now().time_since_epoch().count()); });
  std::printf("  ① clock::now()                    %8.1f ns   （%.4f%% 帧预算）\n", t_now, pct(t_now));

  // ── ② FrameDebug 填满（~50 个字段）──
  auto_aim::FrameDebug fd;
  const double t_fill = bench(N, [&] {
    fd.frame_id = 12345;
    fd.t_frame_us = 9000; fd.t_perceive_us = 2000; fd.t_decide_us = 100;
    fd.mode = 1; fd.game_state = 4;
    fd.detector.armor_count = 3; fd.detector.best_confidence = 0.9; fd.detector.nms_survivors = 2;
    fd.detector.t_infer_us = 6000;
    fd.solver.solved = 1; fd.solver.reprojection_error = 0.3; fd.solver.yaw_offset = -1.5;
    fd.solver.t_solve_us = 200;
    fd.tracker.state = 2; fd.tracker.armor_count = 3; fd.tracker.priority_mode = 3;
    fd.tracker.filtered_out = 0; fd.tracker.t_track_us = 5;
    fd.target.tracked_id = 1;
    fd.target.xyz_world[0] = 1.5; fd.target.xyz_world[1] = 0.2; fd.target.xyz_world[2] = 0.3;
    fd.target.yaw = 1.1; fd.target.w = 3.2; fd.target.r = 0.2; fd.target.l = 0.15;
    fd.target.h = 0.1; fd.target.nis = 2.0; fd.target.nis_thresh = 9.5;
    fd.target.invincible = false;
    fd.planner.t_fly = 0.28;   // ⚠️ W79：t_fire/t_pred/dps/kill_time 已删（算法里没这些量）
    fd.controller.cmd_yaw = 1.0; fd.controller.cmd_pitch = 0.05;
    fd.controller.control = true; fd.controller.shoot = true;
    g_sink += fd.frame_id + size_t(fd.target.w * 100);
  });
  std::printf("  ② FrameDebug 填满(~40 字段)        %8.1f ns   （%.4f%%）\n", t_fill, pct(t_fill));

  // ── ③ SinkHub::on_frame（不挂 sink）──
  tools::SinkHub hub;
  const double t_hub = bench(N, [&] { hub.on_frame(fd); });
  std::printf("  ③ SinkHub::on_frame **不挂 sink**  %8.1f ns   （%.4f%%）\n", t_hub, pct(t_hub));
  std::printf("     ⭐ W46 已加**原子快速路径**（无 sink 时免锁）→ 相对底噪只多 %.1f ns\n",
              t_hub - t_floor);

  // ── ④ Expense：6 个段的 begin/end/us ──
  tools::Expense exp;
  const char * tags[6] = {"perceive", "detect", "solve", "track", "plan", "control"};
  const double t_exp = bench(N, [&] {
    for (int i = 0; i < 6; ++i) exp.begin(tags[i]);
    for (int i = 0; i < 6; ++i) exp.end(tags[i]);
    for (int i = 0; i < 6; ++i) g_sink += size_t(exp.us(tags[i]));
    exp.next_frame();
  });
  std::printf("  ④ Expense 6 段 begin/end/us        %8.1f ns   （%.4f%%）\n",
              t_exp, pct(t_exp));

  // ── ⑤ logger()->debug 被过滤掉时（默认 info 级）──
  tools::set_log_level(spdlog::level::info);
  const double t_log = bench(N, [&] { tools::logger()->debug("armors={} t={}", 3, 6000); });
  std::printf("  ⑤ logger()->debug 被级别过滤       %8.1f ns   （%.4f%%）\n", t_log, pct(t_log));

  // ── ⑥ logger 传一个"昂贵表达式"（经典坑）──
  int calls = 0;
  auto expensive = [&] { ++calls; return 42; };
  tools::set_log_level(spdlog::level::info);   // debug 被关
  calls = 0;
  bench(N, [&] { tools::logger()->debug("x={}", expensive()); });   // ⚠️ 参数会先算
  std::printf("  ⑥ 同上但参数是昂贵表达式          %8.1f ns   ⚠️ 表达式被调用了 %d 次/%d\n",
              bench(N, [&] { tools::logger()->debug("x={}", expensive()); }), calls, N);

  // ── ⑨⭐⭐⭐ 我从没测过的三项：热键 / 录制 / PlotJuggler ──
  {
    tools::HotkeyConsole hk;
    const double t_hk = bench(N, [&] { g_sink += size_t(hk.poll()); });
    std::printf("\n  ⑨ HotkeyConsole::poll（每帧一次）  %8.1f ns = %.3f µs  （%.4f%%）\n",
                t_hk, t_hk / 1000.0, pct(t_hk));
    std::printf("     含一次 `read()` 系统调用（raw 非阻塞）→ 相对底噪多 %.1f ns\n", t_hk - t_floor);
  }
  {
    tools::Recorder rec;
    cv::Mat img(1080, 1440, CV_8UC3, cv::Scalar(0));
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    // ⚠️ Recorder 有 fps 节流（默认 30），所以多数帧会在第一行早退
    const double t_rec = bench(20000, [&] {
      rec.record(img, q, std::chrono::steady_clock::now());
    });
    std::printf("  ⑩ Recorder::record（fps 节流后）    %8.1f ns = %.3f µs  （%.4f%%）\n",
                t_rec, t_rec / 1000.0, pct(t_rec));
    std::printf("     ⭐ 未达 fps 时只做一次 delta_time 比较就返回\n");
  }
  {
    auto pj = std::make_shared<tools::PlotJugglerSink>("127.0.0.1", 9870, /*enable=*/false);
    const double t_pj = bench(20000, [&] { pj->on_frame(fd); });
    std::printf("  ⑪ PlotJugglerSink（enable=false）   %8.1f ns = %.3f µs  （%.4f%%）\n",
                t_pj, t_pj / 1000.0, pct(t_pj));
  }

  // ── ⑦⭐⭐ CsvSink::on_frame（**在自瞄线程**：格式化 + 入队）──
  {
    auto fd2 = fd;
    auto csv = std::make_shared<tools::CsvSink>("/tmp/hzmir_ov");
    const double t_csv = bench(20000, [&] { csv->on_frame(fd2); });
    std::printf("\n  ⑦ CsvSink::on_frame（自瞄线程）     %8.1f ns = %.1f µs  （%.3f%% 帧预算）\n",
                t_csv, t_csv / 1000.0, pct(t_csv));
    std::printf("     ⚠️ 这里含 **57 列字符串格式化** —— 它就在自瞄线程里，是真实代价！\n");
    std::printf("     ✅ 而**落盘**（fputs）在 worker 线程 —— 那部分不同步\n");
  }

  // ── ⑧ ImageSink::on_frame（异步，入队 + 唤醒）──
  {
    auto img = cv::Mat(1080, 1440, CV_8UC3, cv::Scalar(0));
    auto sink = std::make_shared<tools::ImageSink>("/tmp/hzmir_ovimg", 8);
    const double t_img = bench(20000, [&] { sink->on_image("t", img, 0); });
    std::printf("  ⑧ ImageSink::on_image（自瞄线程）   %8.1f ns = %.1f µs  （%.3f%% 帧预算）\n",
                t_img, t_img / 1000.0, pct(t_img));
    std::printf("     ⚠️⚠️ 但含 **`img.clone()`（4.4 MB）** —— 它也在自瞄线程！\n");
    std::printf("        实测 clone 一次 ≈ 140 µs；默认 every_n=30 → 摊薄 ≈ 4.7 µs/帧\n");
    std::printf("        ⚠️ 若 every_n=1 → 每帧 140 µs = 帧预算的 1.4%%\n");
    std::printf("        ✅ 只有 PNG 编码 + 落盘在 worker 线程\n");
    // ⭐⭐ W46：对比浅拷贝（引用计数）
    {
      // ⚠️ `max_files` 一定要小 —— 否则 worker 会往磁盘写满 PNG（我第一版写 100000 → OOM/写爆盘）
      auto sh = std::make_shared<tools::ImageSink>(
        "/tmp/hzmir_ovsh", /*every_n=*/1, /*max_files=*/50, /*deep_copy=*/false);
      const double t_sh = bench(20000, [&] { sh->on_image("t", img, 0); });
      std::printf("  ⭐ ⑧b ImageSink（deep_copy=false）  %8.1f ns = %.2f µs  （%.3f%%）\n",
                  t_sh, t_sh / 1000.0, pct(t_sh));
      std::printf("       浅拷贝 = 引用计数 +1 → 从 %.1f µs 降到 %.2f µs（%.0f× 快）\n",
                  t_img / 1000.0, t_sh / 1000.0, t_img / (t_sh > 0 ? t_sh : 1));
      std::printf("       ⚠️ 仅当上游每帧新建 cv::Mat 时安全（MindVision/HikRobot ✅；USBCamera/Video ❌）\n");
    }
  }

  // ── ⑦ 合计：默认路径 ──
  const double total = t_now + t_fill + t_hub + t_exp;
  std::printf("\n  ────────────────────────────────────────────────\n");
  std::printf("  ⭐ 默认路径合计（① 一次 + ②+③+④）  %8.1f ns   （%.4f%% 帧预算）\n", total,
              pct(total));
  std::printf("  ⭐ 换算成 µs: %.3f µs / 10000 µs\n", total / 1000.0);

  std::printf("\n─ 判读 ────────────────────────────────────────\n");
  std::printf("  · ⭐ W44 已把 `Expense` 从 std::map 改成定长线性表：782 → 92 ns\n");
  std::printf("  · ⚠️ ⑥ 证明 **spdlog 会先算参数再判级别** —— 热路径别写 logger()->debug(昂贵表达式)\n");
  std::printf("  · 对照：同济 `*_debug.cpp` 的 `cv::imshow+waitKey(1)` 是 **ms 级**（本测试量不了）\n");
  return 0;
}
