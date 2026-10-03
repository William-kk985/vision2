#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "utils/log/logger.hpp"
#include "utils/math/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() / 57.3;    // degree to rad
  second_tolerance_ = yaml["second_tolerance"].as<double>() / 57.3;  // degree to rad
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
}

Shooter::ShootResult Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  // ⭐⭐ W98：**统一出口** —— 有 4 处早返回（无控制/无目标/自动开火关/正常）。
  //   原来用 `last_decision_` 成员 + 返回 bool（W72 的半个 Result 模式）；
  //   现在收成 lambda 直接产出 `ShootResult`，**结构上不可能忘填**。
  auto finish = [](bool fire) {
    ShootResult r;
    r.fire = fire;
    r.dbg.should_fire = fire;
    return r;
  };

  if (!command.control) { auto r = finish(false); r.dbg.blocked_by_no_control = true; return r; }
  if (targets.empty())  { auto r = finish(false); r.dbg.blocked_by_no_target = true; return r; }
  if (!auto_fire_)      { auto r = finish(false); r.dbg.blocked_by_auto_fire_off = true; return r; }

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;

  if (
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    aimer.debug_aim_point.valid) {
    auto r = finish(true);
    r.dbg.tolerance = tolerance;
    last_command_ = command;
    return r;
  }

  // ⭐ W72：判据没过 —— 明确标出原因（便于定位"为什么不开火"）
  auto r = finish(false);
  r.dbg.tolerance = tolerance;
  r.dbg.blocked_by_filter = true;
  last_command_ = command;
  return r;
}

}  // namespace auto_aim