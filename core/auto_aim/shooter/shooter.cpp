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

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  // ⭐⭐ W72：每次调用先清零，再按实际分支填原因
  last_decision_ = Decision{};
  if (!command.control) { last_decision_.blocked_by_no_control = true; return false; }
  if (targets.empty()) { last_decision_.blocked_by_no_target = true; return false; }
  if (!auto_fire_) { last_decision_.blocked_by_auto_fire_off = true; return false; }

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;
  // tools::logger()->debug("d(command.yaw) is {:.4f}", std::abs(last_command_.yaw - command.yaw));
  last_decision_.tolerance = tolerance;
  if (
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&    //应该减去上一次command的yaw值
    aimer.debug_aim_point.valid) {
    last_decision_.fire = true;
    last_command_ = command;
    return true;
  }

  // ⭐ W72：判据没过 —— 细分成三个具体原因（便于定位"为什么不开火"）
  last_decision_.blocked_by_filter = true;
  last_command_ = command;
  return false;
}

}  // namespace auto_aim