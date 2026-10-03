#ifndef AUTO_AIM__SHOOTER_HPP
#define AUTO_AIM__SHOOTER_HPP

#include <string>

#include "core/types.hpp"
#include "core/auto_aim/planner/legacy.hpp"
#include "core/auto_aim/shooter/shooter_debug.hpp"   // ⭐ W98

namespace auto_aim
{
class Shooter
{
public:
  Shooter(const std::string & config_path);

  /// ⭐⭐ W98：返回 `ShootResult`（是否开火 + 调试快照）
  ///
  /// ## 从"访问器"到"返回值"
  /// W72 已经加了 `Decision` + `last_decision()` 访问器 —— 那是**半个** Result 模式：
  /// 数据是带出来了，但**要主循环主动去取** ⚠️（忘了取就静默为默认值）。
  /// ⇒ 现在 `shoot()` 直接**返回**它，与 Detector/Tracker/Aimer 一致。
  struct ShootResult
  {
    bool fire = false;
    ShooterDebug dbg;
  };
  ShootResult shoot(
    const io::Command & command, const auto_aim::Aimer & aimer,
    const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos);


private:
  io::Command last_command_;
  double judge_distance_;
  double first_tolerance_;
  double second_tolerance_;
  bool auto_fire_;
};
}  // namespace auto_aim

#endif  // AUTO_AIM__SHOOTER_HPP