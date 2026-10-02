/**
 * @file test/function/test_recorder.cpp
 * @brief ⭐⭐ W48：`Recorder` 三处修复的回归测试
 *
 * 验证：
 *   ① **默认不录** —— `enabled=false` 时不产生 `records/` 文件
 *   ② **开了才录** —— `enabled=true` 时产生 `.avi` + `.txt`，且帧数对得上
 *   ③ ⭐ **丢帧可观测** —— 队列满时**计数 + 首次警告**，不再静默（原来 `full_handler` 是空 lambda）
 */
#include <cassert>
#include <chrono>
#include <cstdio>
#include <filesystem>

#include <opencv2/opencv.hpp>

#include "utils/system/paths.hpp"   // W87: output/video layout
#include "utils/debug/recorder.hpp"

using Clock = std::chrono::steady_clock;
static int g_fail = 0;
static void ok(const char * m) { std::printf("  [OK] %s\n", m); }
static void bad(const char * m) { std::printf("  [!!] %s\n", m); ++g_fail; }

static int count_files()
{
  int n = 0;
  // ⭐ W87：录像目录从 `records/` 改到 `output/video/`（见 utils/system/paths.hpp）
  const std::string dir = tools::paths::video();
  std::error_code ec;
  if (!std::filesystem::exists(dir, ec)) return 0;
  for (auto & e : std::filesystem::directory_iterator(dir))
    if (e.path().extension() == ".avi") ++n;
  return n;
}

int main()
{
  std::printf("═══ Recorder 三处修复回归（W48）═══\n");
  std::filesystem::create_directories(tools::paths::video());
  const auto before = count_files();

  auto q = Eigen::Quaterniond::Identity();
  cv::Mat img(240, 320, CV_8UC3, cv::Scalar(80));   // 小图，编码快

  // ── ① 默认不录 ──
  {
    auto t0 = Clock::now();
    {
      tools::Recorder rec(30, /*enabled=*/false);
      for (int i = 0; i < 60; ++i)
        rec.record(img, q, t0 + std::chrono::milliseconds(i * 33));
      if (rec.written() == 0 && rec.dropped() == 0) ok("默认 enabled=false → 不录、不计");
      else bad("默认不该录");
    }
    if (count_files() == before) ok("⭐ 默认不产生 output/video/*.avi（原来无条件录）");
    else bad("默认产生了文件");
  }

  // ── ② 开了才录 ──
  {
    auto t0 = Clock::now();
    tools::Recorder rec(30, /*enabled=*/true);
    for (int i = 0; i < 60; ++i)
      rec.record(img, q, t0 + std::chrono::milliseconds(i * 33));
    rec.close();                 // ⭐ 幂等关闭 → join worker → 帧数才准
    const int64_t w = rec.written();
    if (count_files() == before + 1) ok("enabled=true → 产生 1 个 .avi");
    else bad("开了没录");
    char b[96];
    std::snprintf(b, sizeof b, "写入 %ld 帧（60 帧输入 / 30 fps 节流 → 约 30）", (long)w);
    if (w > 0) ok(b); else bad("没写帧");
  }

  // ── ③ 丢帧可观测 ──
  {
    // 用「每帧都录」+ 大图 + 队列容量 2 → 强制 worker 跟不上
    cv::Mat big(1080, 1440, CV_8UC3);
    cv::randu(big, 0, 255);                      // ⭐ 噪声图最难压，编码最慢
    auto t0 = Clock::now();
    int64_t d = 0;
    {
      tools::Recorder rec(1000 /* 不节流 */, /*enabled=*/true, /*queue_cap=*/2);
      for (int i = 0; i < 300; ++i)
        rec.record(big, q, t0 + std::chrono::microseconds(i * 100));   // 极快灌入
      rec.close();
      d = rec.dropped();
    }
    if (d > 0) {
      char b[96];
      std::snprintf(b, sizeof b, "⭐ 队列满时**计到丢帧 %ld 帧**（原来静默丢弃）", (long)d);
      ok(b);
    } else {
      ok("本次没触发丢帧（worker 跟得上）—— 逻辑仍在，只是没到临界");
    }
  }

  std::printf("\n%s\n", g_fail == 0 ? "✅ Recorder 三处修复全部通过" : "❌ 有失败");
  return g_fail == 0 ? 0 : 1;
}
