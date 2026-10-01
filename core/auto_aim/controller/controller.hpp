/**
 * @file core/auto_aim/controller/controller.hpp
 * @brief 角色：控制器（doc 09 §4.0）—— 把决策结果转成下发的 io::Command
 *
 * 职责（原本散在 aimer.cpp / standard_mpc.cpp / infantry.cpp 里）：
 *   · 输出迟滞（deadband）—— 避免末端抖动
 *   · 限幅（yaw/pitch 速度与加速度）
 *   · vel/acc 装配（同济 MPI 要前馈）
 *
 * ⚠️ W4 只做**最小可用版**（passthrough + deadband），完整版见 doc 10 W13。
 */
#ifndef HZMIR_CORE_AUTO_AIM_CONTROLLER_CONTROLLER_HPP
#define HZMIR_CORE_AUTO_AIM_CONTROLLER_CONTROLLER_HPP

#include "core/types.hpp"

namespace auto_aim
{

struct ControllerConfig
{
  double yaw_deadband = 0.3 * CV_PI / 180.0;    // 输出迟滞（rad）
  double pitch_deadband = 0.2 * CV_PI / 180.0;  // 输出迟滞（rad）
  bool enable_deadband = true;
};

class Controller
{
public:
  explicit Controller(const ControllerConfig & cfg = {}) : cfg_(cfg) {}

  /// @brief 决策输出 -> 下发指令（含迟滞）
  io::Command to_command(bool control, bool shoot, double yaw, double pitch);

  void reset() { has_last_ = false; }

private:
  ControllerConfig cfg_;
  bool has_last_ = false;
  double last_yaw_ = 0, last_pitch_ = 0;
};

}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_CONTROLLER_CONTROLLER_HPP
