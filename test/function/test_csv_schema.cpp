/**
 * @file test/function/test_csv_schema.cpp
 * @brief ⭐⭐ CSV **schema 护栏**：表头 ↔ `row()` 必须严格对应
 *
 * ## 防的是什么
 * `CsvSink` 的表头是**手写的 59 列字符串**（7 段拼接），`row()` 也是**手写的 `<<` 链**。
 * 如果给 `FrameDebug` 加了字段、在表头加了列名，**但忘了在 `row()` 里加 `<<`**：
 * ```
 * frame_id,t_frame_us,...,NEW_COL,next_col,...
 *    0   ,   60123   ,...,  <-- 这里少了一个值 → 之后**全部左移一位**
 * ```
 * ⚠️ **后果：整个 CSV 静默变成垃圾**（列名和值错位），而你看数据完全看不出来。
 *
 * 本测试用**真实产出**对比：写几帧 → 读回 → 断言表头列数 == 数据列数。
 */
#include <cassert>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "utils/debug/csv_sink.hpp"

static int g_fail = 0;
static void ok(const char * m) { std::printf("  [OK] %s\n", m); }
static void bad(const char * m) { std::printf("  [!!] %s\n", m); ++g_fail; }

static std::vector<std::vector<std::string>> read_csv(const std::string & p)
{
  std::vector<std::vector<std::string>> out;
  std::ifstream f(p);
  std::string line;
  while (std::getline(f, line)) {
    if (line.empty()) continue;
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string c;
    while (std::getline(ss, c, ',')) cells.push_back(c);
    out.push_back(std::move(cells));
  }
  return out;
}

int main()
{
  std::printf("═══ CSV schema 护栏 ═══\n");
  const std::string prefix = "csvschema_test";

  {
    tools::CsvSink csv(prefix);
    for (int i = 0; i < 3; ++i) {
      auto_aim::FrameDebug d;
      d.frame_id = static_cast<uint32_t>(i);
      d.t_frame_us = 100000 + i;
      d.detector.armor_count = i;
      d.target.tracked_id = i + 1;
      d.planner.t_fly = 0.1 * i;
      csv.on_frame(d);
      csv.on_series("k", i, static_cast<double>(i));
    }
  }   // 析构 → flush

  const auto rows = read_csv(prefix + "_frames.csv");
  if (rows.size() < 4) { bad("frames.csv 行数不足"); return 1; }
  const size_t hn = rows[0].size();

  // ── ① 表头列数 == 每行数据列数 ──
  {
    int bad_cnt = 0;
    for (size_t r = 1; r < rows.size(); ++r)
      if (rows[r].size() != hn) {
        bad((std::string("第 ") + std::to_string(r) + " 行列数 " + std::to_string(rows[r].size()) +
             " != 表头 " + std::to_string(hn) + "（⚠️ 会串列！）").c_str());
        ++bad_cnt;
      }
    if (bad_cnt == 0) {
      char b[128];
      std::snprintf(b, sizeof b, "⭐ 表头 %zu 列 == 每行数据 %zu 列（**没有串列**）", hn, hn);
      ok(b);
    }
  }

  // ── ② 表头无重复列名（重复 = 复制粘贴错误）──
  {
    bool dup = false;
    for (size_t i = 0; i < hn; ++i)
      for (size_t j = i + 1; j < hn; ++j)
        if (rows[0][i] == rows[0][j]) {
          bad((std::string("表头有重复列名: ") + rows[0][i]).c_str());
          dup = true;
        }
    if (!dup) ok("⭐ 表头无重复列名");
  }

  // ── ③ 表头无空列名（拼接处漏了逗号会出空名）──
  {
    bool empty = false;
    for (const auto & c : rows[0])
      if (c.empty()) { bad("表头有空列名（拼接处可能漏了逗号）"); empty = true; break; }
    if (!empty) ok("⭐ 表头无空列名");
  }

  // ── ④ 已知关键列存在 ──
  {
    const std::vector<std::string> must = {"frame_id",   "t_frame_us",           "det_t_infer_us",
                                           "tgt_nis",    "trk_priority_mode",     "pln_t_fly",
                                           "sht_should_fire", "ctl_shoot",       "buff_t_us"};
    int miss = 0;
    for (const auto & m : must) {
      bool found = false;
      for (const auto & c : rows[0]) if (c == m) { found = true; break; }
      if (!found) { bad((std::string("缺关键列 ") + m).c_str()); ++miss; }
    }
    if (miss == 0) ok("⭐ 9 个关键列都在（detector/solver/tracker/target/planner/shooter/controller/buff 各至少 1）");
  }

  // ── ⑤ series.csv 结构：key,t_us,value ──
  {
    const auto sr = read_csv(prefix + "_series.csv");
    if (sr.empty() || sr[0].size() != 3) bad("series.csv 表头不是 3 列 key,t_us,value");
    else if (sr[0][0] != "key" || sr[0][1] != "t_us" || sr[0][2] != "value")
      bad("series.csv 表头名字不对");
    else {
      int bad_cnt = 0;
      for (size_t r = 1; r < sr.size(); ++r)
        if (sr[r].size() != 3) ++bad_cnt;
      if (bad_cnt == 0) ok("⭐ series.csv 每行 3 列（key,t_us,value）");
      else bad("series.csv 有行不是 3 列（W46 那个尾标记 bug 的同类）");
    }
  }

  std::remove((prefix + "_frames.csv").c_str());
  std::remove((prefix + "_series.csv").c_str());
  std::printf("\n%s\n", g_fail == 0 ? "✅ CSV schema 护栏全部通过" : "❌ 有失败");
  return g_fail == 0 ? 0 : 1;
}
