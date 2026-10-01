// test/virtual/test_video_replay.cpp —— ⭐ 录像回放验证（零硬件）
//
// ① 合成一段视频（移动的方块 = 假装甲板）+ 同名 .txt 位姿文件
// ② 通过 io::Camera 工厂（camera_name: video）读回来
// ③ 通过 io::ReplayBoard 回放云台四元数 + 记录下发
#include <cassert>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include "io/board/board.hpp"
#include "io/board/replay.hpp"
#include "io/camera/camera.hpp"
#include "io/camera/video.hpp"

static int passed = 0;
static void ok(const char * w) { std::printf("  [OK] %s\n", w); passed++; }

int main()
{
  const std::string avi = "synth_demo.avi";
  const std::string txt = "synth_demo.txt";
  const int W = 640, H = 480, N = 60, FPS = 30;

  // ═══ ① 造合成视频 + 位姿文件 ═══
  {
    cv::VideoWriter vw(avi, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'), FPS, {W, H});
    assert(vw.isOpened());
    std::ofstream pf(txt);
    for (int i = 0; i < N; ++i) {
      cv::Mat img(H, W, CV_8UC3, cv::Scalar(30, 30, 30));
      int cx = 80 + i * 8;                       // 方块向右移动
      cv::rectangle(img, {cx, H / 2 - 40}, {cx + 30, H / 2 + 40}, {255, 255, 255}, -1);
      cv::rectangle(img, {cx + 30, H / 2 - 40}, {cx + 60, H / 2 + 40}, {255, 255, 255}, -1);
      vw.write(img);
      // 位姿：绕 z 轴缓慢旋转
      double ang = 0.05 * i;
      pf << (i / double(FPS)) << ' ' << std::cos(ang / 2) << " 0 0 " << std::sin(ang / 2) << '\n';
    }
    vw.release();
    ok("合成 60 帧视频 + 位姿文件（`.avi` + 同名 `.txt`）");
  }

  // ═══ ② 直接构造 VideoCamera ═══
  {
    io::VideoCamera cam(avi, 1.0);
    assert(cam.ok());
    assert(std::abs(cam.fps() - FPS) < 1.0);
    std::printf("     fps=%.1f frames=%d\n", cam.fps(), cam.frame_count());
    ok("VideoCamera 打开成功，读到 fps / 帧数");

    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    int n = 0;
    while (true) {
      cam.read(img, t);
      if (img.empty()) break;
      ++n;
    }
    assert(n == N);
    ok("逐帧读完 60 帧（合成时间戳按帧率推进）");
  }

  // ═══ ③ 通过 io::Camera 工厂（camera_name: video）═══
  {
    const std::string cfg = "synth_cam.yaml";
    YAML::Node y;
    y["camera_name"] = "video";
    y["video_path"] = avi;
    y["video_speed"] = 1.0;
    std::ofstream(cfg) << y;
    io::Camera cam(cfg);                        // ← 走工厂
    cv::Mat img;
    std::chrono::steady_clock::time_point t;
    cam.read(img, t);
    assert(!img.empty());
    ok("io::Camera 工厂识别 camera_name=video（不需要 exposure_ms）");
    std::remove(cfg.c_str());
  }

  // ═══ ④ ReplayBoard：回放云台四元数 + 记录下发 ═══
  {
    io::ReplayBoard board(txt, io::GimbalMode::AUTO_AIM, 22.0f);
    assert(board.pose_ok());
    assert(board.mode() == io::GimbalMode::AUTO_AIM);
    assert(std::abs(board.state().bullet_speed - 22.0f) < 1e-6);
    ok("ReplayBoard：固定档位 AUTO_AIM + 弹速 22");

    // 第一帧应为单位四元数（ang=0）
    auto q0 = board.q(std::chrono::steady_clock::now());
    assert(std::abs(q0.w() - 1.0) < 1e-6);
    // 第 11 帧应有明显旋转（ang=0.5 rad）
    for (int i = 0; i < 10; ++i) board.q(std::chrono::steady_clock::now());
    auto q10 = board.q(std::chrono::steady_clock::now());
    assert(std::abs(q10.z()) > 0.2);
    std::printf("     q0.w=%.4f  q10.z=%.4f（回放生效）\n", q0.w(), q10.z());
    ok("云台四元数按行回放（第 0 帧单位，第 10 帧有旋转）");

    board.send(true, true, 0.1f, 0.5f, 1.0f, -0.05f, 0.2f, 0.5f);
    board.send(false, false, 0, 0, 0, 0, 0, 0);
    assert(board.sent_log().size() == 2);
    assert(board.sent_log()[0].control && board.sent_log()[0].fire);
    assert(std::abs(board.sent_log()[0].yaw - 0.1f) < 1e-6);
    ok("send() 只记录不上发（2 条记录，字段正确）—— 便于事后分析下发序列");
  }

  std::remove(avi.c_str());
  std::remove(txt.c_str());
  std::printf("\n✅ 录像回放全部通过（%d 项）—— ⭐ 主链路现在能零硬件跑\n", passed);
  return 0;
}
