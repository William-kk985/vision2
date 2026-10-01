/**
 * @file core/auto_aim/trajectory/trajectory.hpp
 * @brief 角色：弹道解算（doc 09 §4.0）
 *
 * 槽位有两个实现（运行期选）：
 *   - `ideal`    : 斜抛无阻力（同济 tools/trajectory.hpp 的解析解）
 *   - `rk4_drag` : RK4 含空气阻力（待从「你的 ballistic_solver」搬入，见 doc 10 W10）
 *
 * ⚠️ 命名说明：`tools::Trajectory` 是**结果结构体**（在 core/types.hpp 契约层），
 *    这里是**解算器**。两者同名但不同物。
 */
#ifndef HZMIR_CORE_AUTO_AIM_TRAJECTORY_TRAJECTORY_HPP
#define HZMIR_CORE_AUTO_AIM_TRAJECTORY_TRAJECTORY_HPP

#include <memory>
#include <string>

#include "core/types.hpp"

namespace auto_aim
{

class ITrajectory
{
public:
  virtual ~ITrajectory() = default;

  /// @brief 解算弹道：v0 初速(m/s)、d 水平距离(m)、h 高度差(m)
  virtual tools::Trajectory solve(double v0, double d, double h) const = 0;

  virtual const char * name() const = 0;
};

/// @brief 各实现通过这个函数暴露自己（实现类保持匿名，不污染头文件）
std::unique_ptr<ITrajectory> make_ideal_trajectory();       // ideal.cpp
std::unique_ptr<ITrajectory> make_rk4_trajectory(bool big);  // rk4_drag.cpp
/// ⭐ table.cpp：离线 RK4 建表 + 在线查表（取 RK4 精度 + ideal 速度）
std::unique_ptr<ITrajectory> make_table_trajectory(bool big, const std::string & table_path = "");
/// ⭐ tongji_linear.cpp：同济英雄分支的**线性阻力解析解**（`analytical=true` 用原式）
std::unique_ptr<ITrajectory> make_tongji_linear_trajectory(bool analytical = true);

/// @brief 工厂：按 yaml 的 trajectory_impl 选实现
///   "ideal"      斜抛无阻力（同济原版，微秒级）
///   "rk4_drag"   17mm RK4 含阻力（毫秒级，见 test_ballistic 实测）
///   "rk4_drag_42" 42mm 版本（英雄）
///   "table"       ⭐ 查表版（17mm）
///   "table_42"    ⭐ 查表版（42mm）
///   "tongji_linear" / "tongji_linear_log"  ⭐ 同济英雄的线性阻力（原式 / 正确对数式）
/// @throw std::runtime_error 未知实现时抛异常（★ 不静默回退）
std::unique_ptr<ITrajectory> make_trajectory(const std::string & impl);

}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_TRAJECTORY_TRAJECTORY_HPP
