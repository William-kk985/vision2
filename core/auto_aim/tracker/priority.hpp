/**
 * @file core/auto_aim/tracker/priority.hpp
 * @brief ⭐ 目标优先级（从 `omniperception::Decider` 的 4 个 PriorityMap 捞出）
 *
 * **背景（C18）**：`tracker.cpp` 用 `armor.priority` 做排序与目标切换，
 * 而唯一给 `priority` 赋值的地方是 `omniperception::Decider::set_priority()`
 * —— 自瞄链路不调用它 → 读到**未初始化值** → 排序/切换行为随机。
 * 本模块把优先级能力搬进自瞄链路。
 *
 * **同时修 D11**：原 `decider.cpp:25` 把 yaml 键从 `mode` 改名成 `priority_mode`，
 * 但 3 个 yaml 仍写 `mode:` → **静默回退 MODE_ONE** → 哨兵实际执行「3、4 号步兵优先」
 * 而不是配置的「1、2 号英雄优先」。
 * 现在 `parse_priority_mode()` **同时接受两种键名**，且未知值**抛异常**（不静默回退）。
 */
#ifndef HZMIR_CORE_AUTO_AIM_TRACKER_PRIORITY_HPP
#define HZMIR_CORE_AUTO_AIM_TRACKER_PRIORITY_HPP

#include <list>
#include <string>
#include <unordered_map>

#include <yaml-cpp/yaml.h>

#include "core/types.hpp"

namespace auto_aim
{

using PriorityMap = std::unordered_map<ArmorName, ArmorPriority>;

enum PriorityMode
{
  MODE_ONE = 1,
  MODE_TWO,
  MODE_THREE,
  MODE_FOUR
};

/// @brief 4 个优先级表（内容原样来自 omniperception::Decider）
const PriorityMap & priority_map(PriorityMode mode);

/// @brief 就地给 armors 打上 priority（原 omniperception::Decider::set_priority）
void set_priority(std::list<Armor> & armors, PriorityMode mode);

/// @brief 从 yaml 解析模式：⭐ 同时接受 `priority_mode` 与 `mode` 两种键
/// @throw std::runtime_error 值非法时（★ 不静默回退）
PriorityMode parse_priority_mode(const YAML::Node & yaml, PriorityMode fallback = MODE_ONE);

const char * to_string(PriorityMode mode);

}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_TRACKER_PRIORITY_HPP
