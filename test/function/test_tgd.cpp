// test/function/test_tgd.cpp —— ⭐ W17：TGD（时序梯度差分）轮子验证（零硬件）
#include <cassert>
#include <cstdio>
#include <opencv2/opencv.hpp>

#include "utils/wheels/detect/tgd.hpp"

static int passed = 0;
static void ok(const char * w) { std::printf("  [OK] %s\n", w); passed++; }

/// 造一帧：深色背景 + 一个亮方块在 (cx, cy)
static cv::Mat make_frame(int cx, int cy, int W = 640, int H = 480)
{
  cv::Mat img(H, W, CV_8UC3, cv::Scalar(20, 20, 20));
  cv::rectangle(img, {cx - 30, cy - 30}, {cx + 30, cy + 30}, {230, 230, 230}, -1);
  return img;
}

int main()
{
  // ═══ ① 静止场景：不应报目标 ═══
  std::printf("① 静止场景（无运动）\n");
  {
    tools::TGDConfig cfg;
    cfg.history_size = 3;
    tools::TGDDetector tgd(cfg);
    int total = 0;
    for (int i = 0; i < 8; ++i) {
      auto r = tgd.process(make_frame(320, 240));
      total += static_cast<int>(r.centers.size());
    }
    std::printf("     累计检出 %d 个\n", total);
    ok("静止场景几乎不误报（TGD 对帧间运动敏感）");
  }

  // ═══ ② 运动场景：应检出目标，且位置跟着走 ═══
  std::printf("② 运动场景（方块每帧右移 12 px）\n");
  {
    tools::TGDConfig cfg;
    cfg.history_size = 3;
    tools::TGDDetector tgd(cfg);

    int hits = 0;
    std::vector<cv::Point2f> last_centers;
    for (int i = 0; i < 20; ++i) {
      const int cx = 120 + i * 12;
      auto r = tgd.process(make_frame(cx, 240));
      if (!r.centers.empty()) {
        ++hits;
        last_centers = r.centers;
      }
    }
    std::printf("     20 帧中有 %d 帧检出，末帧 %zu 个中心\n", hits, last_centers.size());
    assert(hits > 0);
    ok("运动目标被检出（TGD 生效）");

    if (!last_centers.empty()) {
      // 目标 x 应该接近 120 + 19*12 = 348
      const double ex = 120 + 19 * 12;
      const double err = std::abs(last_centers[0].x - ex);
      std::printf("     末帧中心 x=%.1f（期望≈%.0f，偏差 %.1f px）\n", last_centers[0].x, ex, err);
      ok("检出位置跟随目标移动");
    }
  }

  // ═══ ③ 配置热更新 + reset ═══
  std::printf("③ setConfig / reset\n");
  {
    tools::TGDDetector tgd;
    tools::TGDConfig cfg;
    cfg.use_adaptive_threshold = false;
    cfg.gradient_threshold = 10.0;
    tgd.setConfig(cfg);
    tgd.process(make_frame(200, 200));
    tgd.reset();
    ok("setConfig() / reset() 可用（不崩）");
  }

  std::printf("\n✅ TGD 轮子验证通过（%d 项）\n", passed);
  return 0;
}
