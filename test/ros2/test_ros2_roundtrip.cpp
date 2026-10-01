/**
 * @file test/ros2/test_ros2_roundtrip.cpp
 * @brief ⭐⭐⭐ W39：ROS2 通信层的**自动化往返测试**
 *
 * ## 为什么不是照搬同济的 3 个测试
 * 同济的 `tests/{publish,subscribe,topic_loop}_test.cpp` 都是
 * **手动无限循环的诊断程序**（`while(!exiter.exit())` + `sleep(1s)`），
 * 需要人盯着终端看 —— **不能进 ctest**。
 *
 * 本测试把它们升级为**自动化**：
 * ```
 *   ① 发 EnemyStatusMsg{invincible_enemy_ids=[3,6]} → 轮询 subscribe_enemy_status()
 *      直到收到或超时 → **断言内容一致**
 *   ② 发 AutoaimTargetMsg{target_ids=[4]} → 同理
 *   ③ 上行：用 rclcpp 直接订阅 `auto_aim_target_pos`，再用 ROS2::publish 发一串
 *      已知数据 → **断言外部订阅者收到的字符串内容正确**
 * ```
 * ⭐ 这样「ROS2 层是否真的通」就有了**机器可验证的结论**，而不是"看着像通了"。
 *
 * ⚠️ 需要 `HAS_ROS2`（`-DHZMIR_WITH_ROS2=ON`）；否则本文件不参与构建。
 * ⚠️ 运行需 `ROS_LOG_DIR` 指向**可写**目录（rclcpp 默认写 `~/.ros/log/`）。
 */
#include <atomic>
#include <chrono>
#include <cstdio>
#include <string>
#include <thread>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <sp_msgs/msg/autoaim_target_msg.hpp>
#include <sp_msgs/msg/enemy_status_msg.hpp>
#include <std_msgs/msg/string.hpp>

#include "core/auto_aim/tracker/nav_bridge.hpp"
#include "core/types.hpp"
#include "io/ros2/ros2.hpp"
#include "utils/log/logger.hpp"

using namespace std::chrono_literals;

static int g_fail = 0;
static void ok(const char * msg) { std::printf("  [OK] %s\n", msg); }
static void bad(const char * msg)
{
  std::printf("  [!!] %s\n", msg);
  ++g_fail;
}

/// ⭐ 轮询等待：在 `timeout` 内反复取，直到非空
template <typename Fn>
static std::vector<int8_t> wait_for(Fn && get, std::chrono::milliseconds timeout)
{
  const auto deadline = std::chrono::steady_clock::now() + timeout;
  while (std::chrono::steady_clock::now() < deadline) {
    auto v = get();
    if (!v.empty()) return v;
    std::this_thread::sleep_for(20ms);
  }
  return {};
}

int main(int argc, char ** argv)
{
  // ⚠️ `io::ROS2` 内部会 `rclcpp::init(0, nullptr)` —— 但我们也需要 argc/argv
  //   给 rclcpp 用来解析 ROS 参数，所以在构造 ROS2 之前先 init 一次是不行的
  //   （rclcpp 重复 init 会抛）。这里直接用 ROS2 的构造，额外的 publisher 走
  //   `ROS2::create_publisher` 或 `rclcpp::Node`。
  (void)argc;
  (void)argv;
  std::printf("═══ ROS2 往返测试（W39）═══\n");

  io::ROS2 ros2;
  std::printf("  ROS2 已启动\n");

  // 独立的「外部参与者」节点：既发下行消息，也收上行消息
  auto node = std::make_shared<rclcpp::Node>("hznir_roundtrip_peer");
  auto sub_enemy_pub =
    node->create_publisher<sp_msgs::msg::EnemyStatusMsg>("enemy_status", 10);
  auto sub_autoaim_pub =
    node->create_publisher<sp_msgs::msg::AutoaimTargetMsg>("autoaim_target", 10);

  // ⭐ 上行：我们自己订阅 `auto_aim_target_pos`，看 ROS2::publish 发了什么
  std::atomic<int> up_count{0};
  std::string up_last;
  auto up_sub = node->create_subscription<std_msgs::msg::String>(
    "auto_aim_target_pos", 10, [&](std_msgs::msg::String::SharedPtr m) {
      up_last = m->data;
      ++up_count;
    });

  // rclcpp 的 spin 要在别的线程跑（主线程要做轮询断言）
  std::atomic<bool> stop{false};
  std::thread spin_thread([&]() {
    while (!stop.load() && rclcpp::ok()) {
      rclcpp::spin_some(node);
      std::this_thread::sleep_for(5ms);
    }
  });

  // ── ① 下行：enemy_status ──
  {
    sp_msgs::msg::EnemyStatusMsg m;
    m.invincible_enemy_ids = {3, 6};
    m.timestamp = node->now();
    sub_enemy_pub->publish(m);
    // ⭐ Subscribe2Nav 有 1.5 s 的「失活清空」计时器 → 要**尽快**取
    const auto got = wait_for([&] { return ros2.subscribe_enemy_status(); }, 1200ms);
    if (got.size() == 2 && got[0] == 3 && got[1] == 6) {
      char buf[128];
      std::snprintf(buf, sizeof buf, "下行 enemy_status 收到 {%d, %d}（期望 {3, 6}）",
                    int(got[0]), int(got[1]));
      ok(buf);
    } else {
      char buf[128];
      std::snprintf(buf, sizeof buf, "下行 enemy_status 失败：收到 %zu 个", got.size());
      bad(buf);
    }
  }

  // ── ② 下行：autoaim_target ──
  {
    sp_msgs::msg::AutoaimTargetMsg m;
    m.target_ids = {4};
    m.timestamp = node->now();
    sub_autoaim_pub->publish(m);
    const auto got = wait_for([&] { return ros2.subscribe_autoaim_target(); }, 1200ms);
    if (got.size() == 1 && got[0] == 4) {
      ok("下行 autoaim_target 收到 {4}（期望 {4}）");
    } else {
      char buf[128];
      std::snprintf(buf, sizeof buf, "下行 autoaim_target 失败：收到 %zu 个", got.size());
      bad(buf);
    }
  }

  // ── ③ 上行：auto_aim_target_pos ──
  {
    const Eigen::Vector4d payload{3.33, 4.44, 1.0, 4.0};
    const int before = up_count.load();
    for (int i = 0; i < 20 && up_count.load() == before; ++i) {
      ros2.publish(payload);
      std::this_thread::sleep_for(50ms);
    }
    if (up_count.load() == before) {
      bad("上行 auto_aim_target_pos **没收到任何消息**");
    } else {
      // 期望 "3.330000,4.440000,1.000000,4.000000"
      const bool good = up_last.rfind("3.33", 0) == 0 && up_last.find("4.44") != std::string::npos &&
                        up_last.find("4.000000") != std::string::npos;
      std::printf("      上行原始数据: \"%s\"\n", up_last.c_str());
      if (good) {
        ok("上行 auto_aim_target_pos 内容正确（x,y,1,name+1 逗号拼接）");
      } else {
        bad("上行 auto_aim_target_pos 内容不符（期望 x,y,1,name+1）");
      }
    }
  }

  // ── ④ nav_bridge 与上行的对接（纯函数，不依赖 ROS2）──
  {
    auto_aim::Target t(4.0, 1.0, 0.2, 0.1);
    t.name = auto_aim::ArmorName::sentry;
    std::list<auto_aim::Target> ts{t};
    auto_aim::Armor a;
    a.name = auto_aim::ArmorName::sentry;
    a.xyz_in_gimbal = Eigen::Vector3d(1.5, -2.5, 1.0);
    std::list<auto_aim::Armor> as{a};
    const auto v = auto_aim::target_info_for_nav(as, ts);
    if (v[0] == 1.5 && v[1] == -2.5 && v[2] == 1.0 && v[3] == double(auto_aim::ArmorName::sentry) + 1) {
      ok("nav_bridge → 上行 payload 的字段顺序与 +1 约定正确");
    } else {
      bad("nav_bridge 输出不符");
    }
  }

  stop.store(true);
  spin_thread.join();

  if (g_fail == 0) {
    std::printf("\n✅ ROS2 往返测试全部通过（4 项）\n");
  } else {
    std::printf("\n❌ ROS2 往返测试失败 %d 项\n", g_fail);
  }
  return g_fail == 0 ? 0 : 1;
}
