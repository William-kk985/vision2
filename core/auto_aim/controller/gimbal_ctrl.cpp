/**
 * @file core/auto_aim/controller/gimbal_ctrl.cpp
 * @brief 控制器实现（最小可用版：迟滞 + 组装 io::Command）
 */
#include <cmath>

#include "controller.hpp"

namespace auto_aim
{

/// ⚠️⚠️ **W119 发现：本函数目前是【死代码】—— 没有任何调用点！**
///   实测 `grep -rn "to_command" core/ src/` 只有「本定义 + 头文件声明」。
///   ⭐ 四个主循环（`src/*.cpp`）**都直接调 `board->send(plan.control, plan.fire, ...)`**，
///     绕过了 Controller ⇒ 这里的迟滞/死区**当前根本没生效**。
///   ⭐ 首次下指令的日志因此改到 `io/board/virtual_board.hpp` 的 `send()`（一处覆盖四兵种）。
///   📌 待办：要么主循环改走 `to_command`（迟滞才生效），要么删掉本函数。
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
