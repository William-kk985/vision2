/// @file bench_ours.cpp
/// @brief 本项目【完整自瞄链路】的同线程内联计时，用于与同济对比。
///
/// 与 `bench_tongji.cpp` 口径一致：同一线程内 `detect → track → plan`，
/// 不带窗口、不带 waitKey。目的是消除"本项目在独立规划线程内计时"带来的口径差异
/// （见 docs/20 第 3.3 节）。
#include <algorithm>
#include <map>
#include <chrono>
#include <cstdio>
#include <string>
#include <opencv2/opencv.hpp>

#include "core/auto_aim/detector/det_stats.hpp"
#include "core/auto_aim/detector/detector_slot.hpp"
#include "core/auto_aim/planner/mpc.hpp"
#include "core/auto_aim/solver/solver.hpp"
#include "core/auto_aim/tracker/tracker.hpp"

int main(int argc, char ** argv)
{
  if (argc < 3) { std::printf("usage: %s <config> <video>\n", argv[0]); return 1; }
  const std::string config = argv[1], video_path = argv[2];

  cv::VideoCapture video(video_path);
  if (!video.isOpened()) { std::printf("cannot open video\n"); return 1; }

  // ⭐ 显式对齐主循环：主循环默认 det-stats=true，这里也设 true
  //   （harness 不设会走静态默认值，两边可能不一致 ⇒ 我实测踩到过 det 差 34%）
  // ⭐ 支持 argv[3] 控制 det-stats（默认开，与主循环一致）
  auto_aim::set_det_stats_enabled(!(argc > 3 && std::string(argv[3]) == "nostats"));
  auto_aim::DetectorSlot yolo(config, /*force_yolo=*/true);
  auto_aim::Solver solver(config);
  auto_aim::Tracker tracker(config, solver);
  auto_aim::Planner planner(config);

  std::map<std::string, int> state_count;
  std::vector<double> trk_tracking;   // ⭐ 只统计 tracking 状态（去掉双峰影响）
  std::map<int,int> armor_hist;       // ⭐ 每帧装甲板数量分布（判断输入是否一致）
  cv::Mat img;
  std::vector<double> det, trk, pln;
  int n = 0;
  while (true) {
    video.read(img);
    if (img.empty()) break;
    const auto t = std::chrono::steady_clock::now();

    auto t0 = std::chrono::steady_clock::now();
    auto det_res = yolo.detect(img);   // 与主循环一致：不传 frame_count
    auto t1 = std::chrono::steady_clock::now();
    // ⭐ 与 bench_tongji 口径一致：不做敌我颜色过滤
    //   ⚠️ 否则 demo（蓝装甲板）会被 enemy_color=red 全部滤掉
    //      ⇒ targets 恒为空 ⇒ track/plan 计时为 0（我曾踩到这个坑）
    auto trk_res = tracker.track(det_res.armors, t, /*use_enemy_color=*/false);
    const auto t2trk = std::chrono::steady_clock::now();   // ⭐ track 结束时刻
    armor_hist[static_cast<int>(det_res.armors.size())]++;   // ⭐ 记录输入规模
    state_count[tracker.state()]++;
    if (tracker.state() == "tracking")
      trk_tracking.push_back(std::chrono::duration<double, std::micro>(t2trk - t1).count());
    auto t2 = std::chrono::steady_clock::now();
    if (!trk_res.targets.empty()) planner.plan(trk_res.targets.front(), 22.0);
    auto t3 = std::chrono::steady_clock::now();

    const auto us = [](auto a, auto b) {
      return std::chrono::duration<double, std::micro>(b - a).count();
    };
    if (n >= 20) {
      det.push_back(us(t0, t1));
      trk.push_back(us(t1, t2));
      pln.push_back(us(t2, t3));
    }
    ++n;
  }

  const auto report = [](const char * name, std::vector<double> & v) {
    if (v.empty()) return;
    std::sort(v.begin(), v.end());
    double sum = 0; for (double x : v) sum += x;
    const size_t n = v.size();
    std::printf("  %-6s n=%3zu mean=%6.0f min=%6.0f p25=%6.0f med=%6.0f p75=%6.0f p90=%6.0f max=%7.0f\n",
                name, n, sum / n, v[0], v[n/4], v[n/2], v[n*3/4], v[(size_t)(0.9*(n-1))], v[n-1]);
  };
  std::printf("  ---- 本项目完整链路（同线程内联，无窗口）----\n");
  report("det", det); report("track", trk); report("plan", pln);
  std::printf("  ---- ⭐ 只算 tracking 状态的帧（去双峰）----\n");
  report("trk_trk", trk_tracking);
  if (!det.empty()) {
    std::vector<double> tot(det.size());
    for (size_t i = 0; i < det.size(); ++i) tot[i] = det[i] + trk[i] + pln[i];
    report("TOTAL", tot);
  }
  std::printf("  ---- 每帧装甲板数分布（输入规模）----\n    ");
  for (const auto & [k, v] : armor_hist) std::printf("%d:%d  ", k, v);
  std::printf("\n");
  std::printf("  ---- tracker 状态分布 ----\n");
  for (const auto & [k, v] : state_count) std::printf("    %-12s %d\n", k.c_str(), v);
  return 0;
}
