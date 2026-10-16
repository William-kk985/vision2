/// @file bench_tongji.cpp
/// @brief 上游（同济）【完整自瞄链路】的纯算法计时。
///
/// 链路：YOLO::detect → Tracker::track → Planner::plan
/// 与 `detector_video_test` 的区别：不带窗口、不带 `waitKey(33)`、不连 PlotJuggler，
/// 因此墙钟时间可直接与算法耗时对比。
///
/// 分段计时与我们的 CSV 对齐：det / track / plan 分别报告。
#include <fmt/core.h>

#include <algorithm>
#include <map>
#include <chrono>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "tasks/auto_aim/planner/planner.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"

int main(int argc, char ** argv)
{
  if (argc < 3) { std::printf("usage: %s <config> <video>\n", argv[0]); return 1; }
  const std::string config = argv[1], video_path = argv[2];

  cv::VideoCapture video(video_path);
  if (!video.isOpened()) { std::printf("cannot open video\n"); return 1; }

  auto_aim::YOLO yolo(config, /*debug=*/false);
  auto_aim::Solver solver(config);
  auto_aim::Tracker tracker(config, solver);
  auto_aim::Planner planner(config);

  // ⭐ 统计 tracker 状态分布（判断"track 耗时差异"是否来自状态不同）
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
    auto armors = yolo.detect(img, n);
    auto t1 = std::chrono::steady_clock::now();
    auto targets = tracker.track(armors, t, /*use_enemy_color=*/false);
    const auto t2trk = std::chrono::steady_clock::now();   // ⭐ track 结束时刻
    armor_hist[static_cast<int>(armors.size())]++;   // ⭐ 记录输入规模
    state_count[tracker.state()]++;
    if (tracker.state() == "tracking")
      trk_tracking.push_back(std::chrono::duration<double, std::micro>(t2trk - t1).count());
    auto t2 = std::chrono::steady_clock::now();
    if (!targets.empty()) planner.plan(targets.front(), 22.0);
    auto t3 = std::chrono::steady_clock::now();

    const auto us = [](auto a, auto b) {
      return std::chrono::duration<double, std::micro>(b - a).count();
    };
    if (n >= 20) {   // 跳过预热
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
  std::printf("  ---- 上游完整链路（debug=false，无 waitKey）----\n");
  report("det", det);
  report("track", trk);
  report("plan", pln);
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
