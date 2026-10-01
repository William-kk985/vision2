/**
 * @file core/auto_aim/controller/gimbal_ctrl.cpp
 * @brief 控制器实现（最小可用版：迟滞 + 组装 io::Command）
 */
#include <cmath>

#include "controller.hpp"

namespace auto_aim
{

io::Command Controller::to_command(bool control, bool shoot, double yaw, double pitch)
{
  if (cfg_.enable_deadband && has_last_) {
    // 输出迟滞：变化小于 deadband 就沿用上次（抑制末端抖动）
    if (std::abs(yaw - last_yaw_) < cfg_.yaw_deadband) yaw = last_yaw_;
    if (std::abs(pitch - last_pitch_) < cfg_.pitch_deadband) pitch = last_pitch_;
  }
  last_yaw_ = yaw;
  last_pitch_ = pitch;
  has_last_ = true;

  io::Command cmd{};
  cmd.control = control;
  cmd.shoot = shoot;
  cmd.yaw = yaw;
  cmd.pitch = pitch;
  return cmd;
}

}  // namespace auto_aim
