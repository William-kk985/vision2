/**
 * @file core/auto_aim/tracker/filter.hpp
 * @brief ⭐ 射击过滤器（从 `omniperception::Decider::armor_filter` 捞出）
 *
 * 原实现硬编码 5 条规则在 omniperception 里，**自瞄链路一条都没有**
 *   → `infantry` 会打前哨站、会打无敌目标、会打 5 号板。
 * 现实现：**规则从 yaml 配**，由 `Tracker` 在关联前调用。
 *
 * 5 条规则（编号沿用原实现）：
 *   ① 非敌方颜色                     -> cfg.enemy_color
 *   ② 25 赛季无 5 号装甲板            -> cfg.skip_names
 *   ③ 不打工程（原已注释）            -> cfg.skip_names
 *   ④ 不打前哨站（原硬编码）          -> cfg.skip_names
 *   ⑤ 不打刚复活无敌的                -> cfg.use_invincible
 *   ⑥ 只打上级指定目标（集火指令）     -> cfg.use_auto_aim_target
 */
#ifndef HZMIR_CORE_AUTO_AIM_TRACKER_FILTER_HPP
#define HZMIR_CORE_AUTO_AIM_TRACKER_FILTER_HPP

#include <list>
#include <vector>

#include <yaml-cpp/yaml.h>

#include "core/types.hpp"

namespace auto_aim
{

/// ⭐⭐ **默认值 = 同济行为**（同济只做「① 颜色过滤」这一条）
///   其余规则是我们的**优化建议**，需在 yaml 的 `armor_filter:` 段显式开启。
struct ArmorFilterConfig
{
  Color enemy_color = Color::red;
  std::vector<ArmorName> skip_names = {};   // ⚠️ 同济没有 ②④（默认空 = 不过滤编号）
  bool use_enemy_color = true;              // ① ✅ 同济有
  bool use_invincible = false;              // ⚠️ 同济没有 ⑤（默认关）
  bool use_auto_aim_target = false;         // ⚠️ 同济没有 ⑥（默认关）
};

class ArmorFilter
{
public:
  ArmorFilter() = default;
  explicit ArmorFilter(const ArmorFilterConfig & cfg) : cfg_(cfg) {}

  /// @brief 从 yaml 读 `armor_filter:` 段（缺键用默认值）
  void load(const YAML::Node & yaml);

  /// @brief 就地过滤；@return 过滤后是否为空
  bool apply(
    std::list<Armor> & armors, const InvincibleMask & invincible = {},
    const std::vector<ArmorName> & auto_aim_targets = {}) const;

  const ArmorFilterConfig & config() const { return cfg_; }

  /// @brief 把 "outpost,five" 这类字符串解析成 ArmorName 列表
  static std::vector<ArmorName> parse_names(const std::string & csv);

private:
  ArmorFilterConfig cfg_;
};

}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_TRACKER_FILTER_HPP
