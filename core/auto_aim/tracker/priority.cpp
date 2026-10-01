#include "priority.hpp"

#include <stdexcept>

#include "utils/log/logger.hpp"

namespace auto_aim
{

namespace
{
// ── 4 个优先级表（原样搬自 omniperception::Decider，见 decider.hpp:65-109）──

// Mode 1: 3/4 号步兵优先
const PriorityMap MODE1 = {
  {ArmorName::one, ArmorPriority::second},   {ArmorName::two, ArmorPriority::forth},
  {ArmorName::three, ArmorPriority::first},  {ArmorName::four, ArmorPriority::first},
  {ArmorName::five, ArmorPriority::third},   {ArmorName::sentry, ArmorPriority::third},
  {ArmorName::outpost, ArmorPriority::fifth},{ArmorName::base, ArmorPriority::fifth},
  {ArmorName::not_armor, ArmorPriority::fifth}};

// Mode 2: 2 号优先
const PriorityMap MODE2 = {
  {ArmorName::two, ArmorPriority::first},    {ArmorName::one, ArmorPriority::second},
  {ArmorName::three, ArmorPriority::second}, {ArmorName::four, ArmorPriority::second},
  {ArmorName::five, ArmorPriority::second},  {ArmorName::sentry, ArmorPriority::third},
  {ArmorName::outpost, ArmorPriority::third},{ArmorName::base, ArmorPriority::third},
  {ArmorName::not_armor, ArmorPriority::third}};

// Mode 3: 先打英雄(1、2 号) -> 再打哨兵 -> 再打 3 号步兵 -> 其他 fourth
const PriorityMap MODE3 = {
  {ArmorName::one, ArmorPriority::first},    {ArmorName::two, ArmorPriority::first},
  {ArmorName::sentry, ArmorPriority::second},{ArmorName::three, ArmorPriority::third},
  {ArmorName::four, ArmorPriority::forth},   {ArmorName::five, ArmorPriority::forth},
  {ArmorName::outpost, ArmorPriority::forth},{ArmorName::base, ArmorPriority::forth},
  {ArmorName::not_armor, ArmorPriority::forth}};

// Mode 4: 先打哨兵 -> 再打英雄(1、2 号) -> 再打 3 号步兵 -> 其他 fourth
const PriorityMap MODE4 = {
  {ArmorName::sentry, ArmorPriority::first}, {ArmorName::one, ArmorPriority::second},
  {ArmorName::two, ArmorPriority::second},   {ArmorName::three, ArmorPriority::third},
  {ArmorName::four, ArmorPriority::forth},   {ArmorName::five, ArmorPriority::forth},
  {ArmorName::outpost, ArmorPriority::forth},{ArmorName::base, ArmorPriority::forth},
  {ArmorName::not_armor, ArmorPriority::forth}};
}  // namespace

const PriorityMap & priority_map(PriorityMode mode)
{
  switch (mode) {
    case MODE_ONE:   return MODE1;
    case MODE_TWO:   return MODE2;
    case MODE_THREE: return MODE3;
    case MODE_FOUR:  return MODE4;
  }
  throw std::runtime_error("未知的 PriorityMode");
}

void set_priority(std::list<Armor> & armors, PriorityMode mode)
{
  if (armors.empty()) return;
  const auto & m = priority_map(mode);
  for (auto & a : armors) {
    auto it = m.find(a.name);
    a.priority = (it != m.end()) ? it->second : ArmorPriority::fifth;
  }
}

PriorityMode parse_priority_mode(const YAML::Node & yaml, PriorityMode fallback)
{
  // ⭐ D11 修复：两种键名都认。
  // ⚠️ 安全护栏：`mode` 这个名字太通用，可能被别的语义占用（你的 yaml 里
  //    `use_relative_mode` / `ti_adaptive_mode` 就是例子）。
  //    → **只有它的值恰好是整数 1~4 时才当作优先级**，否则忽略。
  const YAML::Node node = yaml["priority_mode"] ? yaml["priority_mode"] : yaml["mode"];

  if (node && !yaml["priority_mode"] && yaml["mode"]) {
    int probe = -1;
    try {
      probe = node.as<int>();
    } catch (const std::exception &) {
      tools::logger()->warn(
        "[Priority] yaml 里的 `mode` 不是整数（值语义与优先级无关）→ 忽略，用默认 {}", 
        static_cast<int>(fallback));
      return fallback;
    }
    if (probe < 1 || probe > 4) {
      tools::logger()->warn(
        "[Priority] yaml 里的 `mode: {}` 不在 1~4 → 忽略（它可能是别的语义），用默认 {}",
        probe, static_cast<int>(fallback));
      return fallback;
    }
  }

  if (!node) {
    tools::logger()->warn(
      "[Priority] yaml 里既没有 priority_mode 也没有 mode → 用默认 {}（MODE_ONE）",
      static_cast<int>(fallback));
    return fallback;
  }

  const int v = node.as<int>();
  if (v < 1 || v > 4)
    throw std::runtime_error(
      "[Priority] priority_mode/mode 值非法: " + std::to_string(v) + "（应为 1~4）");

  if (!yaml["priority_mode"] && yaml["mode"])
    tools::logger()->warn(
      "[Priority] yaml 用的是旧键名 `mode: {}`（同济曾改名为 priority_mode 导致静默回退 MODE_ONE）"
      " —— 本次已正确识别，但建议改成 priority_mode",
      v);

  return static_cast<PriorityMode>(v);
}

const char * to_string(PriorityMode mode)
{
  switch (mode) {
    case MODE_ONE:   return "MODE_ONE(3/4号步兵优先)";
    case MODE_TWO:   return "MODE_TWO(2号优先)";
    case MODE_THREE: return "MODE_THREE(英雄优先)";
    case MODE_FOUR:  return "MODE_FOUR(哨兵优先)";
  }
  return "UNKNOWN";
}

}  // namespace auto_aim
