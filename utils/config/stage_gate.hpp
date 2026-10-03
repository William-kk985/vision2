/**
 * @file utils/config/stage_gate.hpp
 * @brief ⭐⭐⭐ **算法独立测试模式**：`--stop-after=<阶段>`（W100 · 原 A2）
 *
 * ## 为什么需要
 * 正常主循环是**一条链**：`取图 → 检测 → 跟踪 → 规划 → 发指令`。
 * 想单独验证其中**一环**时，后面几环会**污染结论**：
 * ```
 * ⚠️ "检测率怎么这么低？"  → 其实是被 Tracker 的颜色过滤器滤掉了
 * ⚠️ "跟踪怎么老丢？"      → 其实是 Planner 的延迟让 target 过期了
 * ⚠️ "命中率变了"          → 分不清是检测/跟踪/规划/控制哪一环的功劳
 * ```
 * ⇒ 需要一个「**跑到哪一步就停**」的开关，把链路**截断**。
 *
 * ## 设计（⚠️ 为什么是"截断"而不是"跳过某一步"）
 * 「跳过中间某一步」在数据流上是**不成立**的（跟踪没有输入就没法跑）。
 * 能截断的只有**前缀**：跑了前 N 步，后面就没有输入。
 * ⇒ 语义就是 `--stop-after=<最后一个要跑的阶段>`。
 *
 * ## 各档位能回答什么问题
 * | 档位 | 跑了什么 | 能回答 |
 * |---|---|---|
 * | `perceive` | 取图 + 姿态 + 坐标变换 | 相机/时间戳/IMU 对不对；帧率多少 |
 * | `detect`   | + 检测 | ⭐ **纯检测率/置信度/推理耗时**（不受跟踪过滤影响） |
 * | `track`    | + 跟踪 | ⭐ **跟踪稳定性**（状态机切换、丢失率），不受规划延迟影响 |
 * | `buff-detect` | 打符检测 + 解算 | 符识别对不对 |
 * | `plan`（默认） | 全跑 | 端到端 |
 *
 * ## 用法
 * ```cpp
 * const auto stop = tools::parse_stop_after(cli.get<std::string>("stop-after"));
 * // ... 主循环里：
 * if (tools::stage_ok(stop, tools::Stage::Detect)) { ...检测... }
 * if (tools::stage_ok(stop, tools::Stage::Track))  { ...跟踪... }
 * if (tools::stage_ok(stop, tools::Stage::Plan))   { target_queue.push(...); }
 * else                                             { target_queue.push(std::nullopt); }
 * ```
 */
#ifndef HZMIR_UTILS_CONFIG_STAGE_GATE_HPP
#define HZMIR_UTILS_CONFIG_STAGE_GATE_HPP

#include <string>

#include "utils/log/logger.hpp"

namespace tools
{

/// 主循环的阶段（**顺序即依赖顺序**）
enum class Stage {
  Perceive = 0,   ///< 取图 + 姿态 + 坐标变换
  Detect = 1,     ///< 检测
  Track = 2,      ///< 跟踪
  BuffDetect = 1, ///< 打符检测（与 Detect 同层，走另一条链）
  BuffSolve = 2,  ///< 打符解算
  Plan = 3,       ///< 规划（默认，= 全跑）
};

/// @brief 解析 `--stop-after`（大小写不敏感；空/未知 = Plan 即全跑）
inline Stage parse_stop_after(const std::string & s)
{
  if (s.empty()) return Stage::Plan;
  std::string t;
  for (char c : s) t += static_cast<char>(std::tolower(static_cast<unsigned char>(c)));

  if (t == "all" || t == "plan" || t == "full") return Stage::Plan;
  if (t == "track")                             return Stage::Track;
  if (t == "detect")                            return Stage::Detect;
  if (t == "perceive" || t == "cam")            return Stage::Perceive;
  if (t == "buff-detect" || t == "buff_detect") return Stage::BuffDetect;
  if (t == "buff-solve" || t == "buff_solve")   return Stage::BuffSolve;

  logger()->warn(
    "[stage] --stop-after 无法识别: \"{}\" → 按默认（全跑）。"
    "可选: perceive / detect / track / buff-detect / buff-solve / plan",
    s);
  return Stage::Plan;
}

/// @brief 该阶段是否应该跑（`stop` 及之前的都跑）
inline bool stage_ok(Stage stop, Stage s) { return static_cast<int>(s) <= static_cast<int>(stop); }

/// @brief 可读描述（启动时打一条，⭐ 避免"以为在跑全链路其实被截断了"）
/// ⚠️ 特意用 if 链而不是 `switch` —— `Detect/BuffDetect`、`Track/BuffSolve`
///    **枚举值相同**（语义上"同层"，只是两条链），`switch` 会报 duplicate case。
inline const char * stage_name(Stage s)
{
  if (s == Stage::Perceive)   return "perceive（只取图）";
  if (s == Stage::Detect)     return "detect（检测，不跟踪）";
  if (s == Stage::Track)      return "track（检测+跟踪，不规划）";
  if (s == Stage::BuffDetect) return "buff-detect（只打符检测）";
  if (s == Stage::BuffSolve)  return "buff-solve（打符检测+解算）";
  return "plan（全链路，默认）";
}

/// @brief 是否被截断了（用于决定要不要 warn / 在 CSV 里标记）
inline bool stage_truncated(Stage s) { return s != Stage::Plan; }

}  // namespace tools

#endif  // HZMIR_UTILS_CONFIG_STAGE_GATE_HPP
