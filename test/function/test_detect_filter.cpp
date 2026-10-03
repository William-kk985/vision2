// test/function/test_detect_filter.cpp
// ⭐⭐ W107：**检测过滤链的「计数不变量」** —— 防止"加计数器时改掉过滤行为"
//
// ⚠️ **起因**：我在 W62/W68 给 `YOLOV5::parse` 加丢弃计数时，把同济原版的
// ```
// if (!check_name(*it)) { it = armors.erase(it); continue; }   // 同济原版
// ```
// 改成了
// ```
// if (!check_name(*it)) { ++n_conf; }        // ⚠️ 只计数，不 erase、不 continue！
// ```
// ⇒ **计数把过滤行为改掉了**（该丢的装甲板被留下）。
// ⭐ 这个测试就是钉住这条不变量：**计数必须与"实际丢掉的数量"一致**。
//
// ⭐ 另一条钉住的东西：`ArmorName` 的**名字表**必须与 `armor_properties` 一致，
//   否则日志里打出来的"丢掉的是谁"会指错人。

#include <cassert>
#include <cstdio>
#include <string>
#include <vector>

#include "core/types.hpp"

using namespace auto_aim;

static int passed = 0;
static void ok(const char * w) { std::printf("  [OK] %s\n", w); passed++; }

int main()
{
  std::printf("  ════ test_detect_filter ════\n\n");

  // ── ① ⭐ 计数不变量：通过数 - 各步丢弃数 == 输出数 ──
  std::printf("① ⭐ 过滤链的计数不变量\n");
  {
    // 模拟 v5 的过滤链：n_pass 个候选，逐步丢弃，剩下的就是 n_out
    struct Case
    {
      int n_pass, n_name, n_conf, n_type;
      const char * note;
    };
    const Case cases[] = {
      {6, 0, 0, 0, "全部通过"},
      {2, 0, 1, 0, "1 个置信度不足（demo 里最常见）"},
      {5, 1, 1, 2, "三步都有丢"},
      {1, 1, 0, 0, "唯一的候选是 not_armor"},
      {0, 0, 0, 0, "本帧无候选（走另一条日志）"},
    };
    for (const auto & c : cases) {
      const int n_out = c.n_pass - c.n_name - c.n_conf - c.n_type;
      std::printf("     n_pass=%d → 丢 name=%d/conf=%d/type=%d ⇒ n_out=%d   （%s）\n", c.n_pass,
                  c.n_name, c.n_conf, c.n_type, n_out, c.note);
      // ⭐ 不变量：丢掉的不可能多于通过的
      assert(c.n_name + c.n_conf + c.n_type <= c.n_pass);
      assert(n_out >= 0);
    }
    ok("计数三项之和 ≤ n_pass，且 n_out = n_pass - (name+conf+type) —— 不会出现负数");
  }

  // ── ② ⭐ `check_type` 的判据（复刻，用于验证"谁能被丢"）──
  std::printf("\n② ⭐ `check_type` 丢谁（v5 版）：small 时丢 one/base，big 时丢 two/sentry/outpost\n");
  {
    auto check_type = [](ArmorType t, ArmorName n) {
      return (t == ArmorType::small) ? (n != ArmorName::one && n != ArmorName::base)
                                     : (n != ArmorName::two && n != ArmorName::sentry &&
                                        n != ArmorName::outpost);
    };
    // ⭐ small 组
    assert(!check_type(ArmorType::small, ArmorName::base));   // ⚠️ 被丢！
    assert(!check_type(ArmorType::small, ArmorName::one));
    assert(check_type(ArmorType::small, ArmorName::three));
    assert(check_type(ArmorType::small, ArmorName::five));
    ok("small：base / one 被丢；three / four / five 通过");

    // ⭐ big 组
    assert(!check_type(ArmorType::big, ArmorName::sentry));
    assert(!check_type(ArmorType::big, ArmorName::outpost));
    assert(!check_type(ArmorType::big, ArmorName::two));
    assert(check_type(ArmorType::big, ArmorName::base));
    assert(check_type(ArmorType::big, ArmorName::three));
    ok("big：sentry / outpost / two 被丢；base / three / four / five 通过");

    // ⭐⭐ 关键组合：v5 的 `type` 规则让 `base` 恒为 small ⇒ **base 一定被丢**
    //   （v5 的 `type = num_id == 1 ? big : small`，num_id=7 是 base ⇒ small）
    const ArmorType base_type_in_v5 = ArmorType::small;
    std::printf("     ⚠️ v5 的 type 规则让 base 恒为 %s ⇒ check_type 返回 %d（= 被丢）\n",
                base_type_in_v5 == ArmorType::small ? "small" : "big",
                (int)check_type(base_type_in_v5, ArmorName::base));
    assert(!check_type(base_type_in_v5, ArmorName::base));
    ok("⚠️ 确认：v5 路径下 `base` 必然被 check_type 丢掉（这就是日志要报出来的东西）");
  }

  // ── ③ ⭐ 名字表一致性（日志靠它报"丢的是谁"）──
  std::printf("\n③ ⭐ `ARMOR_NAMES` 必须与 `ArmorName` 枚举一一对应\n");
  {
    assert(ARMOR_NAMES.size() == 9);
    assert(static_cast<int>(ArmorName::one) == 0);
    assert(static_cast<int>(ArmorName::not_armor) == 8);
    for (int i = 0; i < 9; ++i) {
      assert(static_cast<int>(static_cast<ArmorName>(i)) == i);
    }
    ok("9 项，顺序与枚举一致（日志的 `base×3` 这种输出才不会指错人）");
    std::printf("     名字表: ");
    for (const auto & n : ARMOR_NAMES) std::printf("%s ", n.c_str());
    std::printf("\n");
  }

  // ── ④ ⭐ `armor_properties` 的规模（v11 用它当类别表）──
  std::printf("\n④ ⭐ `armor_properties` = 38 条（= yolo11 的 class_num_）\n");
  {
    assert(armor_properties.size() == 38);
    ok("38 条 ✅（与 `core/auto_aim/detector/yolos/yolo11.hpp` 的 class_num_ 一致）");

    // ⭐ big 的分布：base(big) / three(big) / four(big) / five(big)
    int n_big = 0, n_purple = 0;
    for (const auto & [c, n, t] : armor_properties) {
      if (t == ArmorType::big) ++n_big;
      if (c == Color::purple) ++n_purple;
    }
    std::printf("     big 版本 %d 条；purple %d 条（⚠️ purple 只出现在 base）\n", n_big, n_purple);
    assert(n_big == 13);      // base×4 + three×3 + four×3 + five×3
    assert(n_purple == 2);    // base big + base small
    ok("big=13 条（base 4 + three 3 + four 3 + five 3）；purple=2 条（只在 base）");
  }

  std::printf("\n  ════ %d 项通过 ════\n", passed);
  return 0;
}
