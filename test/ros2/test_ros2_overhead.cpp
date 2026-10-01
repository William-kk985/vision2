/**
 * @file test/ros2/test_ros2_overhead.cpp
 * @brief ⭐⭐⭐ W45：**ROS2 通信在自瞄线程里的真实代价**
 *
 * ## 回答的问题
 * 「**ROS2 也会有这样子的问题吗**」—— 即：ROS2 会不会抢自瞄的资源？
 *
 * ## 会，而且是**两个**问题
 * 1. ⚠️ `publish()` **是同步的** —— `std::to_string` ×4 + 字符串拼接 + DDS 发布
 *    **全在调用者（自瞄）线程里**
 * 2. ⚠️ `io::ROS2` 起 **2 个 spin 线程**，而 **DDS 自己还有更多内部线程**
 *    （接收、事件、发现、异步写）→ **都在抢 CPU**
 *
 * 本测试量化 ①（可测），并打印 ② 的线程数（对照）。
 */
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>
#include <fstream>

#include <Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

#include "io/ros2/ros2.hpp"
#include "utils/log/logger.hpp"

using Clock = std::chrono::steady_clock;

int main(int argc, char ** argv)
{
  (void)argc; (void)argv;
  std::printf("═══ ROS2 通信开销实测（W45）═══\n");
  std::printf("  帧预算 = 10000 µs\n\n");

  // ── ① 只做"拼字符串"（不含 DDS）──
  {
    const int N = 200000;
    Eigen::Vector4d v{3.33, 4.44, 1.0, 4.0};
    std::string s;
    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) {
      s = std::to_string(v[0]) + "," + std::to_string(v[1]) + "," + std::to_string(v[2]) + "," +
          std::to_string(v[3]);
    }
    auto t1 = Clock::now();
    const double ns = std::chrono::duration<double, std::nano>(t1 - t0).count() / N;
    std::printf("  ① 只拼字符串（to_string×4 + 3 次 +）  %8.1f ns = %.2f µs\n", ns, ns / 1000);
  }

  // ── ② 真的 publish（含 DDS）──
  auto ros2 = std::make_unique<io::ROS2>();
  std::this_thread::sleep_for(std::chrono::milliseconds(300));   // 等 DDS 发现完成
  {
    const int N = 5000;
    Eigen::Vector4d v{3.33, 4.44, 1.0, 4.0};
    // 预热
    for (int i = 0; i < 200; ++i) ros2->publish(v);
    auto t0 = Clock::now();
    for (int i = 0; i < N; ++i) ros2->publish(v);
    auto t1 = Clock::now();
    const double ns = std::chrono::duration<double, std::nano>(t1 - t0).count() / N;
    std::printf("  ② io::ROS2::publish（含 DDS）        %8.1f ns = %.2f µs  （%.3f%% 帧预算）\n",
                ns, ns / 1000, ns / 1e7 * 100);
  }

  // ── ③ 本进程的线程数（ROS2 + DDS 起了多少）──
  {
    std::vector<std::thread::id> ids;
    std::ifstream st("/proc/self/status");
    std::string line;
    int threads = 0;
    while (std::getline(st, line))
      if (line.rfind("Threads:", 0) == 0) threads = std::stoi(line.substr(8));
    std::printf("\n  ③ 本进程线程数: %d  ⚠️ 其中 ROS2 起 2 个 spin 线程，DDS 还有内部线程\n", threads);
  }

  std::printf("\n─ 判读 ────────────────────────────────────────\n");
  std::printf("  · ② 若 > 5 µs → **每帧 publish 就是一份真开销**（和 CsvSink 同量级）\n");
  std::printf("  · ③ 的线程数说明：**ROS2 的并发不是「零成本」** —— 它们都在同一个 CPU 池里\n");
  std::printf("  · ⭐ 正确做法：**上行也异步化**（攒一帧→队列→另一个线程 publish）\n");
  return 0;
}
