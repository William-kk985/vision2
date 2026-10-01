#ifndef AUTO_AIM__AIMER_HPP
#define AUTO_AIM__AIMER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>

#include "io/board/can/cboard.hpp"
#include "core/types.hpp"
#include "core/auto_aim/target/target.hpp"

namespace auto_aim
{

struct AimPoint
{
  bool valid;
  Eigen::Vector4d xyza;
};

class Aimer
{
public:
  AimPoint debug_aim_point;
  explicit Aimer(const std::string & config_path);
  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    bool to_now = true);

  io::Command aim(
    std::list<Target> targets, std::chrono::steady_clock::time_point timestamp, double bullet_speed,
    io::ShootMode shoot_mode, bool to_now = true);

  // ═══════════════════════════════════════════════════════════════
  // ⭐⭐ E5：同济兼容开关（**默认 = 同济行为**）
  //
  // 同济原代码：`if (std::abs(target.ekf_x()[8]) <= 2 && ...)`
  //   ⚠️ `x[8]` 是**半径 r**（≈0.2 m）→ `|r| <= 2` **恒成立**
  //      → 其内的「小陀螺 coming/leaving」分支**永不可达**，
  //        且 `comming_angle_` / `leaving_angle_` 两个配置项**完全失效**。
  //   `x[7]` 才是 **w（角速度 rad/s）**，`|w| <= 2 rad/s` = "不算小陀螺"。
  //
  // 按「源代码以同济为准」原则：**默认保持同济的 `x[8]`**，
  // 我们的修正作为**优化建议**，需显式开启。
  // ═══════════════════════════════════════════════════════════════
  static bool tongji_compat();                       ///< 默认 true
  static void set_tongji_compat(bool on);

private:
  double yaw_offset_;
  std::optional<double> left_yaw_offset_, right_yaw_offset_;
  double pitch_offset_;
  double comming_angle_;
  double leaving_angle_;
  double lock_id_ = -1;
  double high_speed_delay_time_;
  double low_speed_delay_time_;
  double decision_speed_;

  AimPoint choose_aim_point(const Target & target);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__AIMER_HPP