/**
 * @file core/auto_aim/trajectory/ideal.cpp
 * @brief 弹道实现①：斜抛无阻力（← 同济 tools/trajectory.cpp 的解析解）
 */
#include <stdexcept>

#include "core/auto_aim/trajectory/trajectory.hpp"

namespace auto_aim
{

namespace
{
class IdealTrajectory : public ITrajectory
{
public:
  tools::Trajectory solve(double v0, double d, double h) const override
  {
    return tools::Trajectory(v0, d, h);  // 同济原实现（core/types.cpp）
  }
  const char * name() const override { return "ideal"; }
};
}  // namespace

std::unique_ptr<ITrajectory> make_ideal_trajectory()
{
  return std::make_unique<IdealTrajectory>();
}

}  // namespace auto_aim
