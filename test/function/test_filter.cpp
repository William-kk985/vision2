// test/function/test_filter.cpp —— W7 横切能力验证（无硬件、无业务依赖）
#include <cassert>
#include <cstdio>
#include <list>
#include <string>

#include <yaml-cpp/yaml.h>

#include "core/types.hpp"
#include "core/auto_aim/tracker/filter.hpp"
#include "core/auto_aim/tracker/priority.hpp"

using namespace auto_aim;

static int passed = 0;
static void ok(const char * what) { std::printf("  [OK] %s\n", what); passed++; }

// 造一块指定 name/color 的假装甲板
static Armor make_armor(ArmorName n, Color c)
{
  Lightbar l, r;
  l.angle = r.angle = 0.1;
  l.length = r.length = 1.0;
  l.width = r.width = 0.2;
  l.ratio = r.ratio = 5.0;
  l.center = {0.f, 0.f};  l.top = {0.f, 0.f};  l.bottom = {0.f, 1.f};
  r.center = {0.5f, 0.f}; r.top = {0.5f, 0.f}; r.bottom = {0.5f, 1.f};
  Armor a(l, r);
  a.name = n;
  a.color = c;
  return a;
}

int main()
{
  // ═══════════════ ① InvincibleMask（无敌位掩码 + 显式 ID 映射）═══════════════
  std::printf("① InvincibleMask\n");
  {
    auto m = InvincibleMask::from_ids({1, 7});           // 英雄 + 前哨站
    assert(m.has(ArmorName::one));
    assert(m.has(ArmorName::outpost));
    assert(!m.has(ArmorName::three));
    ok("from_ids({1,7}) -> has(one)=true, has(outpost)=true, has(three)=false");
    assert(id_to_armor(1) == ArmorName::one);
    assert(id_to_armor(99) == ArmorName::not_armor);      // 未知 id 不越界
    ok("id_to_armor: 1->one, 99->not_armor（不越界）");
  }

  // ═══════════════ ② ArmorFilter：5 条射击规则 ═══════════════
  std::printf("② ArmorFilter\n");
  {
    // ⭐⭐ 默认 = **同济行为**：只做「① 颜色过滤」，其余 4 条规则**默认关闭**
    ArmorFilter f;
    std::list<Armor> armors = {
      make_armor(ArmorName::one,   Color::red),       // 保留
      make_armor(ArmorName::three, Color::blue),      // ① 非敌方 -> 滤
      make_armor(ArmorName::five,  Color::red),       // 同济**不过滤**（25赛季无5号是 22 赛季规则）
      make_armor(ArmorName::outpost, Color::red),     // 同济**不过滤**
      make_armor(ArmorName::four,  Color::red),       // 同济**不过滤**（不判无敌）
    };
    auto inv = InvincibleMask::from_ids({4});
    bool empty = f.apply(armors, inv, {});
    assert(!empty);
    assert(armors.size() == 4);
    std::printf("     默认(同济): 5 块 -> %zu 块（只剩颜色过滤）\n", armors.size());
    ok("⭐ 默认 = 同济行为：只滤颜色（5 -> 4），不动 five/outpost/无敌");

    // ⭐ 显式开启我们的优化 → 5 条规则全生效
    ArmorFilterConfig c_on;
    c_on.skip_names = {ArmorName::five, ArmorName::outpost};
    c_on.use_invincible = true;
    ArmorFilter f_on(c_on);
    std::list<Armor> a_on = {
      make_armor(ArmorName::one,   Color::red),
      make_armor(ArmorName::three, Color::blue),
      make_armor(ArmorName::five,  Color::red),
      make_armor(ArmorName::outpost, Color::red),
      make_armor(ArmorName::four,  Color::red),
    };
    const bool empty2 = f_on.apply(a_on, inv, {});
    assert(!empty2 && a_on.size() == 1 && a_on.front().name == ArmorName::one);
    std::printf("     优化开启:   5 块 -> %zu 块（5 条规则）\n", a_on.size());
    ok("⭐ 显式开启后 5 条规则生效（5 -> 1，只剩 one）");

    // ⑥ 集火指令：只打指定目标（默认关，需显式开）
    ArmorFilterConfig c; c.use_auto_aim_target = true;
    ArmorFilter f2(c);
    std::list<Armor> a2 = {
      make_armor(ArmorName::one, Color::red), make_armor(ArmorName::three, Color::red)};
    f2.apply(a2, {}, {ArmorName::three});
    assert(a2.size() == 1 && a2.front().name == ArmorName::three);
    ok("⑥ 集火指令: 只保留指定目标 three");
  }

  // ═══════════════ ③ PriorityMap：4 张表 + set_priority ═══════════════
  std::printf("③ PriorityMap\n");
  {
    std::list<Armor> armors = {
      make_armor(ArmorName::sentry, Color::red),
      make_armor(ArmorName::one,    Color::red),
      make_armor(ArmorName::three,  Color::red)};
    set_priority(armors, MODE_THREE);          // 英雄(1) 优先
    armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });
    assert(armors.front().name == ArmorName::one);
    ok("MODE_THREE: 英雄(one)排首位");

    set_priority(armors, MODE_FOUR);           // 哨兵优先
    armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });
    assert(armors.front().name == ArmorName::sentry);
    ok("MODE_FOUR: 哨兵(sentry)排首位");

    // ⭐ C18：priority 必须有确定值（不再是未初始化）
    auto fresh = make_armor(ArmorName::three, Color::red);
    assert(fresh.priority == ArmorPriority::fifth);
    ok("C18: 新构造的 Armor::priority == fifth（非未初始化）");
  }

  // ═══════════════ ④ ⭐ D11 修复：同时接受 mode / priority_mode ═══════════════
  std::printf("④ D11（yaml 键名兼容）\n");
  {
    auto y_new = YAML::Load("priority_mode: 3");
    auto y_old = YAML::Load("mode: 3");
    auto y_bad = YAML::Load("priority_mode: 9");
    auto y_none = YAML::Load("other: 1");

    assert(parse_priority_mode(y_new) == MODE_THREE);
    ok("priority_mode: 3  -> MODE_THREE");
    assert(parse_priority_mode(y_old) == MODE_THREE);
    ok("mode: 3           -> MODE_THREE（⭐ 旧键名也认，不再静默回退 MODE_ONE）");
    assert(parse_priority_mode(y_none) == MODE_ONE);
    ok("键名都缺失        -> 默认 MODE_ONE（带 warn）");

    // ⚠️ 安全护栏：`mode` 值不是 1~4 时忽略（避免误读别的语义）
    auto y_str = YAML::Load("mode: fast");
    auto y_big = YAML::Load("mode: 99");
    assert(parse_priority_mode(y_str) == MODE_ONE);
    ok("mode: fast        -> 忽略，用默认（护栏：非整数）");
    assert(parse_priority_mode(y_big) == MODE_ONE);
    ok("mode: 99          -> 忽略，用默认（护栏：不在 1~4）");

    bool threw = false;
    try { parse_priority_mode(y_bad); } catch (const std::exception &) { threw = true; }
    assert(threw);
    ok("priority_mode: 9  -> 抛异常（★ 不静默回退）");
  }

  std::printf("\n✅ W7 横切能力全部通过（%d 项）\n", passed);
  return 0;
}
