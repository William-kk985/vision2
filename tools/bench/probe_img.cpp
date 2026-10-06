/// @file probe_img.cpp
/// @brief 决定性对照：同一进程内，用两种取图路径分别跑 detect()，排除机器状态干扰。
///
/// 背景：`det` 的两种测量给出不同结果（内联 harness 约 7.8 ms，主循环 CSV 约 5.75 ms），
/// 已排除输入图尺寸/格式差异（探针实测两者均为 1440x1080 CV_8UC3）。
/// 本程序把两条路径放在【同一进程、同一时段】内交替执行，若差异消失则说明是
/// 机器状态（频率/热/调度）所致；若差异仍在，则差异在代码路径本身。
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <vector>
#include <opencv2/opencv.hpp>

#include "core/auto_aim/detector/det_stats.hpp"
#include "core/auto_aim/detector/detector_slot.hpp"
#include "io/camera/video.hpp"

static void report(const char * tag, std::vector<double> & v)
{
  if (v.empty()) return;
  std::sort(v.begin(), v.end());
  double sum = 0; for (double x : v) sum += x;
  std::printf("    %-28s n=%zu mean=%6.0fus median=%6.0fus p90=%6.0fus\n", tag, v.size(),
              sum / v.size(), v[v.size() / 2], v[(size_t)(0.9 * (v.size() - 1))]);
}

int main(int argc, char ** argv)
{
  if (argc < 3) { std::printf("usage: %s <config> <video>\n", argv[0]); return 1; }
  const std::string config = argv[1], video = argv[2];
  auto_aim::set_det_stats_enabled(true);

  // 预先读入全部帧（两种路径各读一遍，确保像素完全一致）
  std::vector<cv::Mat> framesA, framesB;
  {
    cv::VideoCapture cap(video);
    cv::Mat f;
    while (cap.read(f) && !f.empty()) framesA.push_back(f.clone());
  }
  {
    io::VideoCamera cam(video, 0.0);
    cv::Mat f;
    std::chrono::steady_clock::time_point t;
    while (true) {
      cam.read(f, t);
      if (f.empty()) break;
      framesB.push_back(f.clone());
    }
  }
  std::printf("    frames: A(cv::VideoCapture)=%zu  B(io::VideoCamera)=%zu\n", framesA.size(),
              framesB.size());

  // 逐像素核对（抽样前 20 帧）
  int diff_frames = 0;
  for (size_t i = 0; i < std::min<size_t>(20, std::min(framesA.size(), framesB.size())); ++i)
    if (cv::countNonZero(framesA[i].reshape(1) != framesB[i].reshape(1)) != 0) ++diff_frames;
  std::printf("    像素不一致的帧数（前 20 帧）: %d\n", diff_frames);

  const auto us = [](auto a, auto b) {
    return std::chrono::duration<double, std::micro>(b - a).count();
  };

  // ── 交替测量：A 用 cv::VideoCapture 的帧，B 用 VideoCamera 的帧 ──
  {
    auto_aim::DetectorSlot yolo(config, true);
    std::vector<double> a, b;
    // 预热
    for (int i = 0; i < 20; ++i) yolo.detect(framesA[i]);
    const size_t n = std::min(framesA.size(), framesB.size());
    for (size_t i = 20; i < n; ++i) {
      auto t0 = std::chrono::steady_clock::now();
      yolo.detect(framesA[i]);
      auto t1 = std::chrono::steady_clock::now();
      yolo.detect(framesB[i]);
      auto t2 = std::chrono::steady_clock::now();
      a.push_back(us(t0, t1));
      b.push_back(us(t1, t2));
    }
    std::printf("  ---- 同一进程内交替测量（同一个 DetectorSlot 实例）----\n");
    report("A: frames from VideoCapture", a);
    report("B: frames from VideoCamera", b);
  }
  return 0;
}
