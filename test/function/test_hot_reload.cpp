// test/function/test_hot_reload.cpp —— ⭐ W21：配置热重载轮子验证
#include <cassert>
#include <chrono>
#include <cstdio>
#include <fstream>
#include <string>
#include <thread>
#include <vector>

#include "utils/config/hot_reloader.hpp"

static int passed = 0;
static void ok(const char * w) { std::printf("  [OK] %s\n", w); passed++; }

static void write_file(const std::string & p, const std::string & s)
{
  std::ofstream(p) << s;
}

int main()
{
  const std::string cfg = "test_hotreload.yaml";

  // ═══ ① ⭐ B11：回调按**注册顺序**（原实现用 std::map → 字典序）═══
  std::printf("① B11：回调顺序\n");
  {
    write_file(cfg, "a: 1\n");
    tools::HotReloader hr(cfg);

    std::vector<std::string> order;
    // 故意用**逆字典序**注册，这样 map 与 vector 的行为会明显不同
    hr.on("zebra", [&](const YAML::Node &) { order.push_back("zebra"); });
    hr.on("mid", [&](const YAML::Node &) { order.push_back("mid"); });
    hr.on("alpha", [&](const YAML::Node &) { order.push_back("alpha"); });
    assert(hr.size() == 3);

    assert(hr.reload());
    std::string joined;
    for (const auto & s : order) joined += s + " ";
    std::printf("     实际顺序: %s（注册顺序: zebra mid alpha）\n", joined.c_str());
    // ⭐ std::map 会给出 alpha mid zebra；vector 给出注册顺序
    assert(order.size() == 3);
    assert(order[0] == "zebra" && order[1] == "mid" && order[2] == "alpha");
    ok("按**注册顺序**调用（zebra→mid→alpha），不再是 map 的字典序（B11 已修）");
  }

  // ═══ ② 解析失败 → 保留旧配置，回调**一次都不调** ═══
  std::printf("② 解析失败保留旧配置\n");
  {
    write_file(cfg, "key: 42\n");
    tools::HotReloader hr(cfg);
    int calls = 0;
    double seen = -1;
    hr.on("c", [&](const YAML::Node & y) { ++calls; seen = y["key"].as<double>(); });

    assert(hr.reload());
    assert(calls == 1 && seen == 42.0);
    ok("正常配置：回调被调用且读到 42");

    write_file(cfg, "key: [unclosed\n");     // 语法错误
    assert(!hr.reload());
    assert(calls == 1);                       // ⭐ 没有再次调用
    ok("语法错误 → reload() 返回 false，**回调不被调用**（旧配置保留）");

    write_file(cfg, "key: 7\n");
    assert(hr.reload());
    assert(calls == 2 && seen == 7.0);
    ok("修好后下次 reload 正常生效（42 → 7）");

    // 文件不存在
    std::remove(cfg.c_str());
    assert(!hr.reload());
    assert(calls == 2);
    ok("文件不存在 → 返回 false，回调不被调用");
  }

  // ═══ ③ ⭐ B12：reload_if_changed（mtime 未变则跳过）═══
  std::printf("③ B12：mtime 门控\n");
  {
    write_file(cfg, "v: 1\n");
    tools::HotReloader hr(cfg);
    int calls = 0;
    int last = -1;
    hr.on("c", [&](const YAML::Node & y) { ++calls; last = y["v"].as<int>(); });

    assert(hr.reload_if_changed());      // 首次：last_mtime_ = -1 → 变化
    assert(calls == 1 && last == 1);
    ok("首次 reload_if_changed() 生效");

    assert(!hr.reload_if_changed());     // mtime 未变 → 跳过
    assert(calls == 1);
    ok("mtime 未变 → 跳过（**回调不再触发**，B12 已修）");

    // 改文件（mtime 至少 1 秒精度 → 等一下）
    std::this_thread::sleep_for(std::chrono::milliseconds(1100));
    write_file(cfg, "v: 2\n");
    assert(hr.mtime() != 0);
    assert(hr.reload_if_changed());
    assert(calls == 2 && last == 2);
    ok("文件改动后 reload_if_changed() 重新触发（1 → 2）");

    // 实际只真正 reload 了 2 次（第 2 次因 mtime 未变被跳过，不计入）
    assert(hr.reload_count() == 2);
    ok("reload_count() 只统计**真正执行**的 reload（跳过的不计）");
  }

  // ═══ ④ 单个组件抛异常不影响其他组件 ═══
  std::printf("④ 组件隔离\n");
  {
    write_file(cfg, "x: 5\n");
    tools::HotReloader hr(cfg);
    int a = 0, c = 0;
    hr.on("good1", [&](const YAML::Node &) { ++a; });
    hr.on("bad", [](const YAML::Node &) { throw std::runtime_error("组件内部错误"); });
    hr.on("good2", [&](const YAML::Node &) { ++c; });
    assert(hr.reload());
    assert(a == 1 && c == 1);
    ok("中间组件抛异常 → 前后组件仍被调用（组件隔离）");
  }

  // ═══ ⑤ YAML 的实际用法（改配置 → 生效）═══
  std::printf("⑤ 实际用法\n");
  {
    write_file(cfg, "armor_filter:\n  skip_names: [\"five\"]\n  use_invincible: false\n");
    tools::HotReloader hr(cfg);
    bool use_inv = true;
    size_t n_skip = 0;
    hr.on("filter", [&](const YAML::Node & y) {
      const auto f = y["armor_filter"];
      if (f["use_invincible"]) use_inv = f["use_invincible"].as<bool>();
      if (f["skip_names"]) n_skip = f["skip_names"].size();
    });
    hr.reload();
    assert(!use_inv && n_skip == 1);
    std::printf("     use_invincible=%d skip_names=%zu\n", int(use_inv), n_skip);
    ok("能读到嵌套段（armor_filter.use_invincible / skip_names）");
  }

  std::remove(cfg.c_str());
  std::printf("\n✅ 配置热重载轮子验证通过（%d 项）\n", passed);
  return 0;
}
