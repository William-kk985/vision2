#include "subscribe2nav.hpp"

#include <sstream>
#include <vector>

namespace io
{

Subscribe2Nav::Subscribe2Nav()
: Node("nav_subscriber"),
  enemy_statue_queue_(1),
  autoaim_target_queue_(1),
  enemy_status_counter_(0),
  autoaim_target_counter_(0)
{
  // ⭐⭐⭐ W108：**建订阅要兜底**。
  //   ⚠️ 实测：没 `source ros2_ws/install/setup.bash`（`sp_msgs` 的类型没注册）时，
  //     `create_subscription` 会抛 `rclcpp::exceptions::RCLError: type_support is null`,
  //     而它在**构造函数**里 ⇒ **没人接 ⇒ `terminate` + 核心转储**，
  //     终端只剩一堆 `rcutils` 的错误转储，**看不出真正原因**（用户实测踩到）。
  //   ⭐ 所以：**接住它，给一条能直接照做的报错**，然后**继续跑**（订阅为空 ⇒
  //     `subscribe_enemy_status()` 返回空 ⇒ 只是"拿不到无敌信息"，**不影响自瞄**）。
  //   ⚠️ 不静默 —— 用 `error` 级别喊出来，并写明修复动作。
  try {
    enemy_status_subscription_ = this->create_subscription<sp_msgs::msg::EnemyStatusMsg>(
      "enemy_status", 10,
      std::bind(&Subscribe2Nav::enemy_status_callback, this, std::placeholders::_1));

    autoaim_target_subscription_ = this->create_subscription<sp_msgs::msg::AutoaimTargetMsg>(
      "autoaim_target", 10,
      std::bind(&Subscribe2Nav::autoaim_target_callback, this, std::placeholders::_1));
  } catch (const std::exception & e) {
    RCLCPP_ERROR(
      this->get_logger(),
      "════ 订阅创建失败，已跳过（自瞄不受影响，只是拿不到 /enemy_status 与 /autoaim_target）════\n"
      "  原因: %s\n"
      "  ⭐ 最常见：**没有 source 工作空间** ⇒ `sp_msgs` 的消息类型没注册。\n"
      "     ⇒ 跑之前先执行：source /opt/ros/humble/setup.bash && "
      "source <repo>/ros2_ws/install/setup.bash\n"
      "  ⭐ 或者：这台机器没有 ROS2 环境 —— 那就用 `--no-ros2` 之类的开关（见 `--print-config`）。",
      e.what());
    return;   // ⚠️ 不再往下建，但【不抛】⇒ 程序继续跑
  }

  RCLCPP_INFO(this->get_logger(), "nav_subscriber node initialized.");
}

Subscribe2Nav::~Subscribe2Nav()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node shutting down.");
}

void Subscribe2Nav::enemy_status_callback(const sp_msgs::msg::EnemyStatusMsg::SharedPtr msg)
{
  enemy_statue_queue_.clear();
  enemy_statue_queue_.push(*msg);

  enemy_status_counter_++;

  if (enemy_status_counter_ >= 2) {
    if (enemy_status_timer_) {
      enemy_status_timer_->cancel();
    }
    enemy_status_timer_ = this->create_wall_timer(std::chrono::milliseconds(1500), [this]() {
      enemy_statue_queue_.clear();
      enemy_status_counter_ = 0;
      RCLCPP_INFO(
        this->get_logger(), "Enemy status queue cleared due to inactivity after two messages.");
    });
  }
}

void Subscribe2Nav::autoaim_target_callback(const sp_msgs::msg::AutoaimTargetMsg::SharedPtr msg)
{
  autoaim_target_queue_.clear();
  autoaim_target_queue_.push(*msg);

  autoaim_target_counter_++;

  if (autoaim_target_counter_ >= 2) {
    if (autoaim_target_timer_) {
      autoaim_target_timer_->cancel();
    }
    autoaim_target_timer_ = this->create_wall_timer(std::chrono::milliseconds(1500), [this]() {
      autoaim_target_queue_.clear();
      autoaim_target_counter_ = 0;
      RCLCPP_INFO(
        this->get_logger(), "Autoaim target queue cleared due to inactivity after two messages.");
    });
  }
}

void Subscribe2Nav::start()
{
  RCLCPP_INFO(this->get_logger(), "nav_subscriber node Starting to spin...");
  rclcpp::spin(this->shared_from_this());
}

std::vector<int8_t> Subscribe2Nav::subscribe_enemy_status()
{
  if (enemy_statue_queue_.empty()) {
    return std::vector<int8_t>();
  }
  sp_msgs::msg::EnemyStatusMsg msg;

  enemy_statue_queue_.back(msg);
  RCLCPP_INFO(
    this->get_logger(), "Subscribe enemy_status at: %d.%09u", msg.timestamp.sec,
    msg.timestamp.nanosec);

  return msg.invincible_enemy_ids;
}

std::vector<int8_t> Subscribe2Nav::subscribe_autoaim_target()
{
  if (autoaim_target_queue_.empty()) {
    return std::vector<int8_t>();
  }
  sp_msgs::msg::AutoaimTargetMsg msg;

  autoaim_target_queue_.back(msg);
  RCLCPP_INFO(
    this->get_logger(), "Subscribe autoaim_target at: %d.%09u", msg.timestamp.sec,
    msg.timestamp.nanosec);

  return msg.target_ids;
}

}  // namespace io