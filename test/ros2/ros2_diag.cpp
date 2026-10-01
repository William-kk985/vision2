/**
 * @file test/ros2/ros2_diag.cpp
 * @brief ⭐ W39：ROS2 **手动诊断工具**（对接真机导航时用）
 *
 * ## 与同济 3 个测试的关系
 * 同济的 `tests/{publish,subscribe,topic_loop}_test.cpp` 是**三个无限循环的手动程序**
 * （各干一件事，都要人盯着终端）。本项目：
 * - ⭐ **自动化验证** → `test/ros2/test_ros2_roundtrip.cpp`（进 ctest，机器判定）
 * - ⭐ **手动诊断** → 本文件（把同济那三个合并成一个带 `--mode` 的工具，**且有 `--duration` 可限时**）
 *
 * ## 用法
 * ```bash
 * # 模拟导航发无敌状态（对接电控/导航前，先用它灌数据看视觉反应）
 * ./test/ros2/ros2_diag --mode=pub-enemy --ids=3,6 --rate=5 --duration=30
 * # 模拟导航发集火指令
 * ./test/ros2/ros2_diag --mode=pub-autoaim --ids=4 --rate=5 --duration=30
 * # 盯着看视觉收到了什么（同济 subscribe_test）
 * ./test/ros2/ros2_diag --mode=watch --duration=30
 * # 往返自检（同济 topic_loop_test）
 * ./test/ros2/ros2_diag --mode=loop --duration=10
 * ```
 */
#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/opencv.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sp_msgs/msg/autoaim_target_msg.hpp>
#include <sp_msgs/msg/enemy_status_msg.hpp>

#include "io/ros2/ros2.hpp"
#include "utils/log/logger.hpp"

using namespace std::chrono_literals;

static std::vector<int8_t> parse_ids(const std::string & s)
{
  std::vector<int8_t> v;
  std::string cur;
  for (size_t i = 0; i <= s.size(); ++i) {
    if (i == s.size() || s[i] == ',') {
      if (!cur.empty()) v.push_back(static_cast<int8_t>(std::stoi(cur)));
      cur.clear();
    } else {
      cur += s[i];
    }
  }
  return v;
}

int main(int argc, char ** argv)
{
  const std::string keys =
    "{help h usage ? | | 输出命令行参数说明}"
    "{mode m         | loop | pub-enemy / pub-autoaim / watch / loop}"
    "{ids            | 3,6 | pub-* 模式下要发的 id 列表（逗号分隔）}"
    "{rate           | 5 | pub-* 模式下的发送频率（Hz）}"
    "{duration       | 10 | ⭐ 运行秒数（0 = 一直跑到 Ctrl-C）}";

  cv::CommandLineParser cli(argc, argv, keys);
  if (cli.has("help")) {
    cli.printMessage();
    return 0;
  }
  const auto mode = cli.get<std::string>("mode");
  const auto ids = parse_ids(cli.get<std::string>("ids"));
  const double rate = cli.get<double>("rate");
  const double duration = cli.get<double>("duration");

  io::ROS2 ros2;
  auto node = std::make_shared<rclcpp::Node>("hznir_ros2_diag");
  std::atomic<bool> stop{false};
  std::thread spin_thread([&]() {
    while (!stop.load() && rclcpp::ok()) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(5ms);
    }
  });

  auto enemy_pub = node->create_publisher<sp_msgs::msg::EnemyStatusMsg>("enemy_status", 10);
  auto autoaim_pub = node->create_publisher<sp_msgs::msg::AutoaimTargetMsg>("autoaim_target", 10);

  const auto t0 = std::chrono::steady_clock::now();
  const auto expired = [&] {
    if (duration <= 0) return false;
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count() >= duration;
  };
  const auto period = std::chrono::microseconds(rate > 0 ? static_cast<int64_t>(1e6 / rate) : 200000);

  tools::logger()->info("[ros2_diag] mode={} ids={} rate={}Hz duration={}s", mode, cli.get<std::string>("ids"),
                        rate, duration);

  size_t n = 0;
  while (!expired()) {
    if (mode == "pub-enemy") {
      sp_msgs::msg::EnemyStatusMsg m;
      m.invincible_enemy_ids = ids;
      m.timestamp = node->now();
      enemy_pub->publish(m);
      if (n % 5 == 0) tools::logger()->info("[ros2_diag] 发 enemy_status ids={}", cli.get<std::string>("ids"));
    } else if (mode == "pub-autoaim") {
      sp_msgs::msg::AutoaimTargetMsg m;
      m.target_ids = ids;
      m.timestamp = node->now();
      autoaim_pub->publish(m);
      if (n % 5 == 0) tools::logger()->info("[ros2_diag] 发 autoaim_target ids={}", cli.get<std::string>("ids"));
    } else if (mode == "watch") {
      const auto inv = ros2.subscribe_enemy_status();
      const auto tgt = ros2.subscribe_autoaim_target();
      if (!inv.empty()) {
        std::string s;
        for (auto i : inv) s += std::to_string(int(i)) + " ";
        tools::logger()->info("[ros2_diag] 收到无敌 ids: {}", s);
      }
      if (!tgt.empty()) {
        std::string s;
        for (auto i : tgt) s += std::to_string(int(i)) + " ";
        tools::logger()->info("[ros2_diag] 收到集火 ids: {}", s);
      }
    } else {  // loop：往返自检（同济 topic_loop_test）
      sp_msgs::msg::EnemyStatusMsg m;
      m.invincible_enemy_ids = ids.empty() ? std::vector<int8_t>{1, 2, 3} : ids;
      m.timestamp = node->now();
      enemy_pub->publish(m);
      const auto got = ros2.subscribe_enemy_status();
      if (n % 5 == 0) tools::logger()->info("[ros2_diag] loop 往返：发出 {} 个 / 收回 {} 个", m.invincible_enemy_ids.size(), got.size());
    }

    ++n;
    std::this_thread::sleep_for(period);
  }

  stop.store(true);
  spin_thread.join();
  tools::logger()->info("[ros2_diag] 结束（共 {} 轮）", n);
  return 0;
}
