/**
 * @file core/auto_aim/trajectory/trajectory.cpp
 * @brief 弹道工厂：**一处注册所有实现**（doc 09 §11.4「逻辑多角色 / 物理同函数」）
 */
#include "core/auto_aim/trajectory/trajectory.hpp"

#include <stdexcept>

namespace auto_aim
{

std::unique_ptr<ITrajectory> make_trajectory(const std::string & impl)
{
  if (impl == "ideal") return make_ideal_trajectory();
  if (impl == "rk4_drag" || impl == "rk4") return make_rk4_trajectory(false);
  if (impl == "rk4_drag_42" || impl == "rk4_42") return make_rk4_trajectory(true);
  if (impl == "table") return make_table_trajectory(false);
  if (impl == "table_42") return make_table_trajectory(true);
  if (impl == "tongji_linear") return make_tongji_linear_trajectory(true);
  if (impl == "tongji_linear_log") return make_tongji_linear_trajectory(false);
  throw std::runtime_error("未知的 trajectory_impl: " + impl);   // ★ 不静默回退
}

}  // namespace auto_aim
