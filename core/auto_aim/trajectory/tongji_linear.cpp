/**
 * @file core/auto_aim/trajectory/tongji_linear.cpp
 * @brief 弹道实现④：**同济英雄分支的线性阻力模型**（轮子 `LinearDragBallistic` 的适配器）
 *
 * ⚠️ 同济该分支把 `pitch` 的符号注释改成了「**抬头为负**」，与 `main`（抬头为正）**相反**。
 *    本适配器**统一回 `ITrajectory` 的约定（抬头为正）**，即对结果取反 ——
 *    这是移植时必须显式处理的**符号约定分歧**。
 */
#include <memory>

#include "core/auto_aim/trajectory/trajectory.hpp"
#include "utils/wheels/ballistic/linear_drag_ballistic.hpp"

namespace auto_aim
{

namespace
{
class TongjiLinearTrajectory : public ITrajectory
{
public:
  explicit TongjiLinearTrajectory(bool analytical) : solver_(analytical) {}

  tools::Trajectory solve(double v0, double d, double h) const override
  {
    tools::Trajectory t(v0, d, h);        // ideal 兜底（含 unsolvable 语义）
    const auto r = solver_.solve(v0, d, h);
    if (!r.solved) return t;              // 与 ideal 一致地处理无解

    t.pitch = r.pitch;                    // ⚠️ 同济 hero 注释「抬头为负」，此处已统一为正
    t.fly_time = r.fly_time;
    return t;
  }

  const char * name() const override { return "tongji_linear"; }

private:
  tools::LinearDragBallistic solver_;
};
}  // namespace

std::unique_ptr<ITrajectory> make_tongji_linear_trajectory(bool analytical)
{
  return std::make_unique<TongjiLinearTrajectory>(analytical);
}

}  // namespace auto_aim
