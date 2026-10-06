/// @file probe_throttle.cpp
/// @brief 验证假设：紧循环连续推理导致 CPU 降频（AVX 密集），插入间隔后单次耗时下降。
///
/// 背景：同一份 detect()，在紧循环 harness 中实测约 7.9 ms，
/// 而在主循环（与跟踪/规划/CSV 交错）中约 5.6 ms —— 交错反而更快，
/// 指向"持续满负载触发降频"。本程序用不同 sleep 间隔验证。
#include <algorithm>
#include <chrono>
#include <cstdio>
#include <thread>
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
  std::printf("    %-22s median=%6.0fus  p90=%6.0fus\n", tag, v[v.size() / 2],
              v[(size_t)(0.9 * (v.size() - 1))]);
}

int main(int argc, char ** argv)
{
  if (argc < 3) { std::printf("usage: %s <config> <video>\n", argv[0]); return 1; }
  const std::string config = argv[1], video = argv[2];
  auto_aim::set_det_stats_enabled(true);

  std::vector<cv::Mat> frames;
  {
    io::VideoCamera cam(video, 0.0);
    cv::Mat f;
    std::chrono::steady_clock::time_point t;
    while (true) { cam.read(f, t); if (f.empty()) break; frames.push_back(f.clone()); }
  }
  auto_aim::DetectorSlot yolo(config, true);
  for (int i = 0; i < 20; ++i) yolo.detect(frames[i]);

  const auto run = [&](int sleep_us) {
    std::vector<double> v;
    for (size_t i = 20; i < frames.size(); ++i) {
      if (sleep_us > 0) std::this_thread::sleep_for(std::chrono::microseconds(sleep_us));
      auto t0 = std::chrono::steady_clock::now();
      yolo.detect(frames[i]);
      auto t1 = std::chrono::steady_clock::now();
      v.push_back(std::chrono::duration<double, std::micro>(t1 - t0).count());
    }
    char tag[64];
    std::snprintf(tag, sizeof(tag), "sleep=%dus", sleep_us);
    report(tag, v);
  };

  std::printf("  ---- 不同迭代间隔下的 detect 耗时 ----\n");
  run(0);      // 紧循环
  run(2000);   // 间隔 2ms
  run(10000);  // 间隔 10ms
  run(0);      // 再紧循环（看是否可复现）
  return 0;
}
