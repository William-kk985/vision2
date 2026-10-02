#include "filter.hpp"

#include <algorithm>
#include <sstream>

#include "utils/log/logger.hpp"
#include "utils/yaml/yaml.hpp"

namespace auto_aim
{

std::vector<ArmorName> ArmorFilter::parse_names(const std::string & csv)
{
  std::vector<ArmorName> out;
  std::stringstream ss(csv);
  std::string tok;
  while (std::getline(ss, tok, ',')) {
    // trim
    auto b = tok.find_first_not_of(" \t");
    auto e = tok.find_last_not_of(" \t");
    if (b == std::string::npos) continue;
    tok = tok.substr(b, e - b + 1);

    auto it = std::find(ARMOR_NAMES.begin(), ARMOR_NAMES.end(), tok);
    if (it == ARMOR_NAMES.end()) {
      tools::logger()->warn("[ArmorFilter] 未知的 armor name: {}", tok);
      continue;
    }
    out.push_back(static_cast<ArmorName>(std::distance(ARMOR_NAMES.begin(), it)));
  }
  return out;
}

void ArmorFilter::load(const YAML::Node & yaml)
{
  // ⭐⭐⭐ W69：**先读顶层 `enemy_color`** —— 与 `Tracker::Tracker()` 保持一致。
  //
  // ⚠️ 原来这里只读 `yaml["armor_filter"]["enemy_color"]`，而 `Tracker` 读的是**顶层**
  //    `yaml["enemy_color"]` ⇒ 两处读**不同位置**，用户设顶层 `enemy_color` 对 filter
  //    **完全无效**（filter 静默用默认 `red`）。
  //
  // 实测（demo.avi，687 帧，蓝色装甲板）：
  //   顶层 enemy_color=red（默认）        → trk_armor_count 非零   0/687  ⚠️ 全被滤掉
  //   顶层 enemy_color=blue               → trk_armor_count 非零   0/687  ⚠️ 依旧无效！
  //   armor_filter: enemy_color=blue      → trk_armor_count 非零 494/687  ✅ 只有段内生效
  //
  // ⇒ 现在：**顶层为准**（和 Tracker 同源），`armor_filter` 段里的值可**显式覆盖**。
  if (yaml["enemy_color"])
    cfg_.enemy_color =
      (yaml["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;

  auto node = yaml["armor_filter"];
  if (!node) {
    tools::logger()->info(
      "[ArmorFilter] 无 armor_filter 段 → 保持同济行为（仅颜色过滤，enemy_color={} 取自顶层）",
      (cfg_.enemy_color == Color::red ? "red" : "blue"));
    return;
  }
  // ⭐ 段内显式覆盖（优先级高于顶层）
  if (node["enemy_color"])
    cfg_.enemy_color = (node["enemy_color"].as<std::string>() == "red") ? Color::red : Color::blue;
  if (node["use_enemy_color"]) cfg_.use_enemy_color = node["use_enemy_color"].as<bool>();
  if (node["use_invincible"]) cfg_.use_invincible = node["use_invincible"].as<bool>();
  if (node["use_auto_aim_target"])
    cfg_.use_auto_aim_target = node["use_auto_aim_target"].as<bool>();
  if (node["skip_names"]) {
    cfg_.skip_names.clear();
    if (node["skip_names"].IsSequence())
      for (const auto & n : node["skip_names"])
        cfg_.skip_names.push_back(static_cast<ArmorName>(
          std::distance(ARMOR_NAMES.begin(),
                        std::find(ARMOR_NAMES.begin(), ARMOR_NAMES.end(), n.as<std::string>()))));
    else
      cfg_.skip_names = parse_names(node["skip_names"].as<std::string>());
  }
  tools::logger()->info(
    "[ArmorFilter] color={} skip={} invincible={} auto_aim_target={}",
    (cfg_.enemy_color == Color::red ? "red" : "blue"), cfg_.skip_names.size(),
    cfg_.use_invincible, cfg_.use_auto_aim_target);
}

bool ArmorFilter::apply(
  std::list<Armor> & armors, const InvincibleMask & invincible,
  const std::vector<ArmorName> & auto_aim_targets) const
{
  if (armors.empty()) return true;

  // ① 非敌方颜色
  if (cfg_.use_enemy_color)
    armors.remove_if([&](const Armor & a) { return a.color != cfg_.enemy_color; });

  // ②③④ 不打哪些编号
  if (!cfg_.skip_names.empty())
    armors.remove_if([&](const Armor & a) {
      return std::find(cfg_.skip_names.begin(), cfg_.skip_names.end(), a.name) !=
             cfg_.skip_names.end();
    });

  // ⑤ 不打无敌
  if (cfg_.use_invincible && invincible.mask != 0)
    armors.remove_if([&](const Armor & a) { return invincible.has(a.name); });

  // ⑥ 只打上级指定（集火指令）
  if (cfg_.use_auto_aim_target && !auto_aim_targets.empty())
    armors.remove_if([&](const Armor & a) {
      return std::find(auto_aim_targets.begin(), auto_aim_targets.end(), a.name) ==
             auto_aim_targets.end();
    });

  return armors.empty();
}

}  // namespace auto_aim
