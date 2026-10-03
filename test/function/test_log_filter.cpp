/// @file test_log_filter.cpp
/// @brief ⭐⭐ W95：按模块过滤日志的单元测试
///
/// 覆盖：解析列表 / 黑名单 / 白名单 / 无标签放行 / 状态描述 / 热键预设循环
#include "utils/log/log_filter.hpp"

#include <cassert>
#include <cstdio>
#include <string>

using namespace tools;

static int failures = 0;
#define CHECK(cond, msg)                                                    \
  do {                                                                      \
    if (!(cond)) { std::printf("  ❌ %s\n", msg); ++failures; }              \
    else         { std::printf("  ✅ %s\n", msg); }                         \
  } while (0)

/// ⭐ 模拟一条日志消息的 payload（spdlog 传给 sink 的已是格式化文本）
static bool pass(const std::string & payload)
{
  return log_module_should_pass(payload.data(), payload.size());
}

int main()
{
  std::printf("  ════ test_log_filter ════\n\n");

  // ── ① parse_module_list ──
  std::printf("  ── parse_module_list ──\n");
  {
    auto v = parse_module_list("yolov5,VirtualBoard,Tracker");
    CHECK(v.size() == 3, "逗号分隔 → 3 项");
    CHECK(v[0] == "yolov5" && v[2] == "Tracker", "顺序与内容正确");

    auto v2 = parse_module_list("a, b ;c\t d");
    CHECK(v2.size() == 4, "逗号/分号/空格/制表符都算分隔");

    CHECK(parse_module_list("").empty(), "空串 → 空列表");
    CHECK(parse_module_list(",,,").empty(), "只有分隔符 → 空列表");
  }

  // ── ② 默认：不过滤 ──
  std::printf("\n  ── 默认（无过滤）──\n");
  clear_log_modules();
  CHECK(!log_filter_active(), "默认不活跃");
  CHECK(pass("[yolov5] any"), "默认放行 yolov5");
  CHECK(pass("[Gimbal] any"), "默认放行 Gimbal");
  CHECK(pass("no tag here"), "默认放行无标签");

  // ── ③ 黑名单 ──
  std::printf("\n  ── 黑名单（--log-off）──\n");
  set_log_modules_off({"yolov5", "VirtualBoard"});
  CHECK(log_filter_active(), "过滤已活跃");
  CHECK(!pass("[yolov5] objectness 通过 6 → 输出 1"), "⭐ yolov5 被静音");
  CHECK(!pass("[VirtualBoard] tx #1"), "⭐ VirtualBoard 被静音");
  CHECK(pass("[Tracker] 新建目标"), "其它模块照常");
  CHECK(pass("[Gimbal] warn"), "其它模块照常 2");
  CHECK(pass("Switch to AUTO_AIM"), "⚠️ 无标签消息【仍放行】（设计如此）");

  // ── ④ 大小写敏感（与源码标签一致）──
  std::printf("\n  ── 大小写 ──\n");
  CHECK(pass("[YOLOV5] 大写"), "⭐ 大小写敏感：YOLOV5 不被 yolov5 规则命中");

  // ── ⑤ 白名单 ──
  std::printf("\n  ── 白名单（--log-only）──\n");
  set_log_modules_only({"Tracker", "Planner"});
  CHECK(pass("[Tracker] 新建目标"), "⭐ 白名单内的放行");
  CHECK(pass("[Planner] 热重载"), "⭐ 白名单内的放行 2");
  CHECK(!pass("[yolov5] 刷屏"), "⭐ 白名单外的【全部丢弃】");
  CHECK(!pass("[Gimbal] warn"), "⭐ 白名单外的【全部丢弃】2");
  CHECK(!pass("[HikRobot] 曝光"), "⭐ 白名单外的【全部丢弃】3");
  CHECK(pass("无标签横幅"), "⚠️ 无标签仍放行");

  // ── ⑥ 白名单会覆盖黑名单 ──
  std::printf("\n  ── 模式互斥 ──\n");
  set_log_modules_off({"Tracker"});
  CHECK(!pass("[Tracker] x"), "先设黑名单 → Tracker 静音");
  set_log_modules_only({"Tracker"});
  CHECK(pass("[Tracker] x"), "⭐ 再设白名单 → Tracker 放行（后设的生效）");

  // ── ⑦ 边界：畸形标签 ──
  std::printf("\n  ── 边界 ──\n");
  set_log_modules_off({"x"});
  CHECK(pass("["), "只有 '[' → 无模块，放行");
  CHECK(pass("[]"), "空标签 → 无模块，放行");
  CHECK(pass("[no close brace"), "没有 ']' → 无模块，放行");
  CHECK(pass("[has space] x"), "⭐ ']' 前有空格 → 不算模块标签，放行");
  CHECK(!pass("[x] ok"), "正常标签仍能被过滤");

  // ── ⑧ 状态描述 & 热键预设 ──
  std::printf("\n  ── 状态 / 热键预设 ──\n");
  clear_log_modules();
  CHECK(log_filter_status().find("关") != std::string::npos, "清空后状态含「关」");

  // ⭐ 索引 0 = 初始状态（不过滤）⇒ 第一次按 n 到索引 1（静音噪音）
  std::string s1 = cycle_log_filter();
  CHECK(log_filter_active() && !pass("[yolov5] x"),
        "⭐ 第 1 次按 n → 静音噪音（yolov5 被静音）");
  CHECK(pass("[Tracker] x"), "   └ 但 Tracker 照常");

  std::string s2 = cycle_log_filter();
  CHECK(pass("[Tracker] x") && !pass("[yolov5] x"),
        "⭐ 第 2 次按 n → 只看关键（白名单）");

  std::string s3 = cycle_log_filter();
  CHECK(!log_filter_active(), "⭐ 第 3 次按 n → 回到「全部」");
  CHECK(pass("[yolov5] x") && pass("[Tracker] x"), "   └ 两个都放行");

  std::string s4 = cycle_log_filter();
  CHECK(log_filter_active() && !pass("[yolov5] x"), "⭐ 第 4 次 → 又开始循环");
  (void)s1; (void)s2; (void)s3; (void)s4;

  std::printf("\n  ════ %s ════\n", failures ? "有失败" : "全部通过");
  return failures ? 1 : 0;
}
