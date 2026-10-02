#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <string>

#include "core/types.hpp"
#include "core/auto_aim/planner/legacy.hpp"

namespace auto_aim
{
class Shooter
{
public:
  Shooter(const std::string & config_path);

  bool shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);

  /// ⭐⭐ W72：**上一次 `shoot()` 的判据快照** —— 解决 `sht_blocked_*` 永远是 0
  ///   原来 `shoot()` 只返回 bool，**外部完全不知道"为什么不开火"**。
  struct Decision
  {
    bool fire = false;
    bool blocked_by_invincible = false;  ///< ⚠️ 无敌在 Tracker 的 filter 层，这里是**透传**的标记
    bool blocked_by_filter = false;      ///< ⭐ 判据没过（command 突变 / 云台没跟上 / 无瞄点）
    bool blocked_by_no_target = false;   ///< 没目标
    bool blocked_by_auto_fire_off = false;
    bool blocked_by_no_control = false;
    double tolerance = 0;                ///< 本次用的容差
  };
  const Decision & last_decision() const { return last_decision_; }

private:
  io::Command last_command_;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  bool auto_fire_;
  Decision last_decision_;   // ⭐ W72
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP