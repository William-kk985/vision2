/**
 * @file core/auto_aim/trajectory/rk4_drag.cpp
 * @brief 弹道实现②：RK4 含空气阻力（⭐ 轮子 `tools::BallisticSolver` 的适配器）
 *
 * 结构：**算法在轮子里，适配在角色里**
 *   轮子  utils/wheels/ballistic/rk4_ballistic.{hpp,cpp}   ← 纯物理，零业务依赖
 *   适配  本文件                                          ← 把结果转成 tools::Trajectory
 */
#include <stdexcept>

#include "core/auto_aim/trajectory/trajectory.hpp"
#include "utils/wheels/ballistic/rk4_ballistic.hpp"

namespace auto_aim
{

namespace
{
class Rk4DragTrajectory : public ITrajectory
{
public:
  /// @param big_bullet true = 42mm（英雄），false = 17mm（步兵/哨兵）
  explicit Rk4DragTrajectory(bool big_bullet)
  : solver_(big_bullet ? tools::BulletType::BIG_42MM : tools::BulletType::SMALL_17MM)
  {
  }

  tools::Trajectory solve(double v0, double d, double h) const override
  {
    const auto r = solver_.solve(v0, d, h, /*use_air_resistance=*/true);

    tools::Trajectory t(v0, d, h);   // 先拿无阻力解做兜底（含 unsolvable 语义）
    if (!r.solved) return t;         // 打不到 → 与理想模型一致地返回 unsolvable

    t.pitch = r.pitch;
    t.fly_time = r.fly_time;
    return t;
  }

  const char * name() const override { return "rk4_drag"; }

private:
  tools::BallisticSolver solver_;
};
}  // namespace

std::unique_ptr<ITrajectory> make_rk4_trajectory(bool big)
{
  return std::make_unique<Rk4DragTrajectory>(big);
}

}  // namespace auto_aim
