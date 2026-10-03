// test/function/test_l3.cpp —— ⭐ W19：L3 图像通道（可视化窗口 / 存图）
#include <cassert>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <memory>
#include <opencv2/opencv.hpp>

#include "core/debug_node.hpp"
#include "utils/debug/debug_sink.hpp"
#include "utils/debug/image_sink.hpp"
#include "utils/debug/window_sink.hpp"

using namespace auto_aim;
namespace fs = std::filesystem;

static int passed = 0;
static void ok(const char * w) { std::printf("  [OK] %s\n", w); passed++; }

static cv::Mat frame(int i)
{
  cv::Mat m(120, 160, CV_8UC3, cv::Scalar(20, 20, 20));
  cv::rectangle(m, {10 + i, 30}, {60 + i, 90}, {230, 230, 230}, -1);
  return m;
}

int main()
{
  // ═══ ① wants_image() 门控：热路径据此决定要不要构造 overlay ═══
  std::printf("① SinkHub::wants_image() 门控\n");
  {
    tools::SinkHub hub;
    assert(!hub.wants_image());
    ok("空 hub → wants_image()=false → 主循环**不构造 overlay**（零成本）");

    auto img_sink = std::make_shared<tools::ImageSink>("test_l3_imgs", 1000, 0);  // 只挂不管
    hub.add(img_sink);
    assert(hub.wants_image());
    ok("挂上 ImageSink → wants_image()=true → 主循环开始构造 overlay");

    hub.remove("image");
    assert(!hub.wants_image());
    ok("移除后 wants_image() 立刻回到 false（热插拔语义）");
  }

  // ═══ ② WindowSink::available()：无显示环境要能识别 ═══
  std::printf("② WindowSink 环境探测\n");
  {
    const bool av = tools::WindowSink::available();
    std::printf("     DISPLAY='%s' → available()=%s\n",
                std::getenv("DISPLAY") ? std::getenv("DISPLAY") : "(未设置)",
                av ? "true" : "false");
    tools::WindowSink ws("test_win");
    ok(av ? "有显示 → 正常开窗" : "无显示 → 只 warn 不开窗，不崩（优雅降级）");
  }

  // ═══ ③ ImageSink：异步落盘 + 节流 + 安全上限 ═══
  std::printf("③ ImageSink（异步 + 节流 + 上限）\n");
  {
    const std::string dir = "test_l3_imgs";
    fs::remove_all(dir);

    const int N = 12;
    {
      tools::ImageSink sink(dir, /*every_n=*/1, /*max_files=*/4);
      for (int i = 0; i < N; ++i) {
        FrameDebug d;
        d.frame_id = static_cast<uint32_t>(i);
        sink.on_frame(d);
        sink.on_image("aim", frame(i), 0);
      }
      // ⚠️ W19 修正：原来在**析构前**断言 saved()==4，但 worker 还没写完 → 不稳定。
      //   现在统计**只能看 on_image 的入队决策**（dropped 是同步的），
      //   落盘数量则等 close() 之后再看。
      assert(sink.dropped() == static_cast<size_t>(N - 4));
      ok("每 1 张存 1 张、上限 4 → 丢弃 8（⭐ 防写爆磁盘的安全网）");
      sink.close();   // ⭐ 等 worker 收尾
      std::printf("     喂 %d 张 → saved=%zu dropped=%zu\n", N, sink.saved(), sink.dropped());
      assert(sink.saved() == 4);
      ok("close() 后落盘数量确定（saved == 4）");
    }
    size_t on_disk = 0;
    for (auto & e : fs::directory_iterator(dir))
      if (e.path().extension() == ".png") ++on_disk;
    std::printf("     磁盘上 .png 文件: %zu\n", on_disk);
    assert(on_disk == 4);
    ok("文件真的落盘了（imwrite 在独立线程里做）");

    // 节流
    fs::remove_all(dir);
    {
      tools::ImageSink sink(dir, /*every_n=*/3, 100);
      for (int i = 0; i < 9; ++i) {
        FrameDebug d;
        d.frame_id = static_cast<uint32_t>(i);
        sink.on_frame(d);
        sink.on_image("aim", frame(i), 0);
      }
      for (int k = 0; k < 100 && sink.saved() < 3; ++k)
        std::this_thread::sleep_for(std::chrono::milliseconds(5));
      std::printf("     每 3 张存 1 张，喂 9 张 → saved=%zu\n", sink.saved());
      assert(sink.saved() == 3);
      ok("every_n 节流生效（9 张只存 3 张）");
    }
    fs::remove_all(dir);
  }

  // ═══ ④ on_image 的默认实现是空操作（不挂 L3 时零成本）═══
  std::printf("④ 默认 on_image 空操作\n");
  {
    tools::SinkHub hub;
    auto csv_like = std::make_shared<tools::NullSink>();
    hub.add(csv_like);
    assert(!hub.wants_image());
    hub.on_image("aim", frame(0), 0);   // 不该有任何事发生
    ok("未挂 L3 时 on_image() 走默认空实现（不会误触发）");
  }

  std::printf("\n✅ L3 图像通道验证通过（%d 项）\n", passed);
  return 0;
}
