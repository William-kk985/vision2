/**
 * @file core/auto_aim/tracker/nav_bridge.hpp
 * @brief ⭐ 导航桥上行的目标信息（从同济 `omniperception::Decider::get_target_info` 捞出）
 *
 * ## 为什么在这里
 * doc 14 §14.3 把 `Decider` 的 10 个方法按「与相机数的关系」分了类：
 * ```
 *   ✅ 与相机数无关（5 个）→ 已捞出 / 本轮捞出
 *      armor_filter / set_priority / get_invincible_armor / get_auto_aim_target
 *      + get_target_info   ← 本文件
 *   ❌ 多相机专属（5 个）→ 归档
 *      decide ×3 / delta_angle / sort
 * ```
 * `get_target_info` 只依赖 `Armor` + `Target`，**与相机数无关**，
 * 所以它不属于已归档的 `omniperception`，而属于自瞄链路。
 *
 * ⚠️ 它只在**哨兵**用得上（上行给导航），但函数本身与兵种无关，故放在这里。
 */
#ifndef HZMIR_CORE_AUTO_AIM_NAV_BRIDGE_HPP
#define HZMIR_CORE_AUTO_AIM_NAV_BRIDGE_HPP

#include <list>

#include <Eigen/Dense>

#include "core/auto_aim/target/target.hpp"
#include "core/types.hpp"

namespace auto_aim
{

/// @brief 打包上行给导航的目标信息（同济 `Decider::get_target_info` 的等价实现）
///
/// 返回 `Eigen::Vector4d{ x, y, 1, name + 1 }`：
///   · `x`/`y` = 目标装甲板在**云台系**下的位置（m）
///   · 第 3 位恒为 `1`（同济原样，含义见通信协议）
///   · 第 4 位 = `ArmorName + 1` ⚠️ **从 1 开始**（同济注释："避免歧义+1(详见通信协议)"）
///
/// 找不到与 `targets.front()` 同名的装甲板时返回**全 0**（同济原样）。
Eigen::Vector4d target_info_for_nav(
  const std::list<Armor> & armors, const std::list<Target> & targets);

}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_NAV_BRIDGE_HPP
