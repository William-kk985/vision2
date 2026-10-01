#include "core/auto_aim/tracker/nav_bridge.hpp"

namespace auto_aim
{

Eigen::Vector4d target_info_for_nav(
  const std::list<Armor> & armors, const std::list<Target> & targets)
{
  if (armors.empty() || targets.empty()) return Eigen::Vector4d::Zero();

  const auto & target = targets.front();

  for (const auto & armor : armors) {
    // ⚠️ 排除未识别目标：`ArmorName` 的枚举里 `not_armor` 是**最后一个（8）**
    //   → `name + 1` 会得到 id **9**，超出上行协议的有效范围（1..8）。
    if (armor.name == ArmorName::not_armor) continue;
    if (armor.name == target.name) {
      return Eigen::Vector4d{
        armor.xyz_in_gimbal[0], armor.xyz_in_gimbal[1], 1,
        static_cast<double>(armor.name) + 1};   // ⚠️ +1：上行 id 从 1 开始
    }
  }

  return Eigen::Vector4d::Zero();
}

}  // namespace auto_aim
