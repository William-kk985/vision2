// test/function/test_debug.cpp —— W8 Debug 数据面验证
#include <cassert>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

#include "core/debug.hpp"
#include "utils/debug/csv_sink.hpp"
#include "utils/debug/debug_sink.hpp"
#include "utils/debug/expense.hpp"
#include "utils/debug/plotjuggler_sink.hpp"

using namespace auto_aim;

static int passed = 0;
static void ok(const char * w) { std::printf("  [OK] %s\n", w); passed++; }

static std::vector<std::string> read_lines(const std::string & p)
{
  std::vector<std::string> v;
  std::ifstream f(p);
  std::string line;
  while (std::getline(f, line)) v.push_back(line);
  return v;
}

int main()
{
  // ═══ ① Expense（L0 常驻耗时，零宏）═══
  std::printf("① Expense（哈工程 DebugExpense 等价物）\n");
  {
    tools::Expense ex;
    ex.begin("perceive");
    volatile double s = 0;
    for (int i = 0; i < 100000; ++i) s += i * 0.5;
    ex.end("perceive");
    ex.begin("decide");
    ex.end("decide");
    assert(ex.us("perceive") > 0);
    ok("begin/end 累计耗时，us('perceive') > 0  ⭐（W19 修复：原来 us() 读的是上一帧→恒 0）");

    // ScopedExpense（RAII）
    { tools::ScopedExpense se(ex, "plan"); }
    assert(ex.us("plan") >= 0);
    ok("ScopedExpense RAII 计时可用");
    ex.next_frame();
    // ⭐ W19 修正语义：next_frame() 之后
    //    us()      = 本帧（已清空）→ 0
    //    last_us() = 上一帧 → > 0
    assert(ex.us("perceive") == 0);
    assert(ex.last_us("perceive") > 0);
    assert(ex.last_us("plan") >= 0);
    assert(ex.us("unknown_tag") == 0 && ex.last_us("unknown_tag") == 0);
    assert(ex.count("perceive") == 0);          // 本帧
    assert(ex.last_summary().find("perceive") != std::string::npos);
    ok("next_frame 语义：us()=本帧 / last_us()=上一帧；count() 也可读（原来恒 0）");
  }

  // ═══ ② SinkHub 热插拔（"像节点一样"的三条性质）═══
  std::printf("② SinkHub（pub/sub + 热插拔）\n");
  {
    tools::SinkHub hub;
    assert(hub.size() == 0);
    ok("空 hub：0 个 sink → on_frame 是空循环（零成本）");

    auto csv = std::make_shared<tools::CsvSink>("test_hub");
    hub.add(csv);
    assert(hub.size() == 1 && hub.has("csv"));
    ok("add(csv) → size=1, has('csv')");

    auto pj = std::make_shared<tools::PlotJugglerSink>("127.0.0.1", 9871, false);  // 关掉避免发包
    hub.add(pj);
    assert(hub.size() == 2);
    assert(hub.remove("csv"));
    assert(hub.size() == 1 && !hub.has("csv"));
    ok("add(plotjuggler) → 2；remove('csv') → 1（⭐ 热插拔 = 从 sinks_ 移除，非 if 判断）");

    assert(hub.toggle(csv));                 // 无 → 加
    assert(!hub.toggle(csv));                // 有 → 删
    ok("toggle() 有则删、无则加（按键语义）");
    hub.add(csv);                            // 留一个写文件
    // 收尾：让 hub 析构前清掉，避免残留
    hub.on_frame(FrameDebug{});
    hub.remove("csv");
    std::remove("test_hub_frames.csv");
    std::remove("test_hub_series.csv");
  }

  // ═══ ③ CsvSink：L0+L1 落盘 + L2 曲线 ═══
  std::printf("③ CsvSink\n");
  {
    const std::string prefix = "test_debug_out";
    {
      tools::CsvSink csv(prefix);
      for (uint32_t i = 0; i < 5; ++i) {
        FrameDebug d;
        d.frame_id = i;
        d.t_frame_us = 100000 + i * 10000;
        d.t_perceive_us = 5000 + i;
        d.t_decide_us = 800 + i;
        d.mode = 1;
        d.detector.armor_count = 2;
        d.detector.best_confidence = 0.9;
        d.tracker.state = 2;
        d.tracker.priority_mode = 3;
        d.tracker.filtered_out = 1;
        d.target.tracked_id = 4;
        d.target.w = 5.5 + i * 0.1;
        d.target.nis = 1.23;
        d.target.invincible = (i == 3);
        d.planner.t_fly = 0.23;
        d.planner.overlap_ratio = 0.6;
        d.planner.kill_time = 1.1;
        d.shooter.should_fire = (i >= 2);
        d.shooter.blocked_by_invincible = (i == 3);
        d.controller.cmd_yaw = -0.05;
        csv.on_frame(d);
        csv.on_series("target.w", d.t_frame_us, d.target.w);
      }
    }  // 析构 → 冲刷
    auto fl = read_lines(prefix + "_frames.csv");
    auto sl = read_lines(prefix + "_series.csv");

    assert(fl.size() == 6);                      // 表头 + 5 行
    ok("frames.csv = 1 行表头 + 5 行数据");
    assert(fl[0].find("frame_id") == 0);
    assert(fl[0].find("tgt_nis") != std::string::npos);
    ok("表头含 frame_id / tgt_nis（⭐ PlotJuggler 落时间轴所需字段齐全）");
    // ⚠️ W19 修正：fl[1] 是 i=0 那一行 → frame_id=0（原来写成 1，Release 下 assert 跳过没暴露）
    assert(fl[1].find("0,100000,5000,800,1,0,") != std::string::npos);
    ok("数据行内容正确（frame_id,t_frame_us,t_perceive_us,t_decide_us,mode,game_state）");

    // invincible 标记：第 3 帧（frame_id=3 -> 文件第 4 行）
    auto & l4 = fl[4];   // i=3
    assert(l4.find("3,130000") != std::string::npos);
    // tgt_invincible 列应为 1
    {
      std::stringstream ss(l4);
      std::string cell;
      int col = 0, inv_col = -1;
      { std::stringstream hs(fl[0]); std::string h; while (std::getline(hs, h, ',')) { if (h == "tgt_invincible") inv_col = col; ++col; } }
      assert(inv_col >= 0);
      std::stringstream ss2(l4);
      for (int c = 0; c <= inv_col && std::getline(ss2, cell, ','); ++c) {}
      assert(cell == "1");
    }
    ok("invincible 列正确（i=3 帧为 1）");

    assert(sl.size() == 6);
    assert(sl[0] == "key,t_us,value");
    assert(sl[1].find("target.w,") == 0);
    ok("series.csv = key,t_us,value + 5 项曲线");

    std::remove((prefix + "_frames.csv").c_str());
    std::remove((prefix + "_series.csv").c_str());
  }

  std::printf("\n✅ W8 Debug 数据面全部通过（%d 项）\n", passed);
  return 0;
}
