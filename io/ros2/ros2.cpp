#include <cstdlib>

#include "utils/system/paths.hpp"   // ⭐ W105：ROS_LOG_DIR 兜底
#include "ros2.hpp"
namespace io
{
ROS2::ROS2()
{
  // ⭐⭐⭐ W105：**把 ROS2 的日志目录指到可写位置（必须在 rclcpp::init 之前）**
  //
  // ⚠️ 坑：ROS2 的 logging 默认写 `~/.ros/log/`（`ROS_HOME`），而那个目录
  //   在很多环境里是**只读**的（沙箱 / 容器 / 只读 home）⇒ `rclcpp::init` 会抛
  //   `spdlog::spdlog_ex: Failed opening file ... Read-only file system`
  //   ⇒ ⚠️ **整个程序 terminate**（不是"日志丢了"那么轻）。
  //   实测：开 `HZMIR_WITH_ROS2` 后**直接跑二进制**（不经 `tools/scripts/run.sh`）必崩。
  //
  // ⭐ 为什么不能只靠 `run.sh` 里那两行 export：
  //   `run.sh` 会设，但**直接跑二进制**（gdb / 手动 / IDE / systemd）就绕过了它。
  //   ⇒ 在程序里再兜一道（**只补没设的**，尊重用户/脚本已设的值）。
  // ⭐ 目录放 `output/logs/` 下（跟本项目其它运行期产物一致，且已 gitignore）。
  {
    const auto set_if_absent = [](const char * key, const std::string & val) {
      if (const char * cur = std::getenv(key); cur && *cur) return;   // 尊重已有设置
      ::setenv(key, val.c_str(), 1);
    };
    // ⚠️ 用相对 CWD 的路径取巧不行 —— `ROS_LOG_DIR` 要求存在或可创建，
    //   所以先确保目录在（`paths::ensure_dir` 递归创建）。
    const std::string ros_log = tools::paths::logs() + "/ros";
    const std::string ros_home = tools::paths::logs() + "/roshome";
    tools::paths::ensure_dir(ros_log);
    tools::paths::ensure_dir(ros_home);
    set_if_absent("ROS_LOG_DIR", ros_log);
    set_if_absent("ROS_HOME", ros_home);
  }

  rclcpp::init(0, nullptr);

  publish2nav_ = std::make_shared<Publish2Nav>();

  subscribe2nav_ = std::make_shared<Subscribe2Nav>();

  publish_spin_thread_ = std::make_unique<std::thread>([this]() { publish2nav_->start(); });

  subscribe_spin_thread_ = std::make_unique<std::thread>([this]() { subscribe2nav_->start(); });
}

ROS2::~ROS2()
{
  rclcpp::shutdown();
  publish_spin_thread_->join();
  subscribe_spin_thread_->join();
}

void ROS2::publish(const Eigen::Vector4d & target_pos) { publish2nav_->send_data(target_pos); }

std::vector<int8_t> ROS2::subscribe_enemy_status()
{
  return subscribe2nav_->subscribe_enemy_status();
}

std::vector<int8_t> ROS2::subscribe_autoaim_target()
{
  return subscribe2nav_->subscribe_autoaim_target();
}

}  // namespace io
