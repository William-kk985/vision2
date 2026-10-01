/**
 * @file test/function/test_nav_bridge.cpp
 * @brief ⭐ W36：`nav_bridge`（从同济 `Decider::get_target_info` 捞出）的单元测试
 *
 * 覆盖：
 *   ① 空输入 → 全 0（同济原样）
 *   ② 命中同名装甲板 → `{x, y, 1, name + 1}`
 *   ③ ⚠️ 第 4 位从 **1** 开始（同济注释"避免歧义+1"）
 *   ④ 找不到同名 → 全 0
 *   ⑤ `InvincibleMask` / `ENEMY_ID_TO_ARMOR` 的边界（与 nav 协议相关）
 */
#include <cassert>
#include <cstdio>
#include <list>

#include "core/auto_aim/target/target.hpp"
#include "core/auto_aim/tracker/nav_bridge.hpp"
#include "core/types.hpp"

using namespace auto_aim;

static Armor make_armor(ArmorName n, double x, double y)
{
  Armor a;
  a.name = n;
  a.color = Color::red;
  a.xyz_in_gimbal = Eigen::Vector3d(x, y, 1.0);
  return a;
}

int main()
{
  std::printf("═══ nav_bridge 单元测试（W36）═══\n");

  // ① 空 armors
  {
    std::list<Armor> armors;
    std::list<Target> targets;
    const auto v = target_info_for_nav(armors, targets);
    assert(v.isZero());
    std::printf("  [OK] 空 armors + 空 targets -> 全 0\n");
  }

  // ②/③ 正常命中（Target 用 4 参数构造器）
  {
    Target t(4.0, 1.0, 0.2, 0.1);
    // ⚠️ 这个构造器**不设 `name`**（原来还是未初始化的垃圾值）。
    //   W36 已给 `name` 补默认值；这里**显式**设成 `three`，
    //   否则下面的断言两边都用同一个垃圾值 → **恒真，测不出东西**。
    t.name = ArmorName::three;
    std::list<Target> targets{t};

    std::list<Armor> armors{
      make_armor(ArmorName::two, 1.11, 2.22),
      make_armor(t.name, 3.33, 4.44),   // ⭐ 与 target 同名
    };
    const auto v = target_info_for_nav(armors, targets);
    assert(v[0] == 3.33 && v[1] == 4.44);
    assert(v[2] == 1.0);
    // ⚠️ 第 4 位 = ArmorName + 1（从 1 开始）
    assert(v[3] == static_cast<double>(ArmorName::three) + 1.0);
    assert(v[3] == 3.0);   // ⭐ 硬编码期望值（three=2 -> 上行 id 3）—— 避免"两边同一变量"式假通过
    std::printf("  [OK] 命中同名装甲板 -> {%.2f, %.2f, %.0f, %.0f}（ArmorName::three=%d -> 上行 id 3）\n",
                v[0], v[1], v[2], v[3], int(ArmorName::three));

    // ⭐ 即使命中项在列表**后面**也要找到（同济是遍历找同名）
    assert(v[0] == 3.33);
    std::printf("  [OK] 遍历查找（同名项在第 2 个位置也能命中）\n");

    // ④ 把 armors 换成不同名 → 全 0
    std::list<Armor> other{make_armor(ArmorName::five, 9.9, 9.9)};
    const auto z = target_info_for_nav(other, targets);
    assert(z.isZero());
    std::printf("  [OK] 无同名装甲板 -> 全 0\n");
  }

  // ⑤ 上行 id 的边界
  {
    // ⭐ 实测发现：`not_armor` 是枚举的**最后一个（8）**，不是 0！
    //   枚举序：one=0 two=1 three=2 four=3 five=4 sentry=5 outpost=6 base=7 not_armor=8
    static_assert(static_cast<int>(ArmorName::one) == 0, "one 必须是 0");
    assert(static_cast<int>(ArmorName::not_armor) == ARMOR_NAMES.size() - 1);
    std::printf("  [OK] ArmorName 序：one=0 ... base=7, **not_armor=%d**（最后一个，不是 0）\n",
                int(ArmorName::not_armor));
    std::printf("  ⚠️  注意：`name + 1` 对 not_armor 会得到 id %d —— 上行时应先排除未识别目标\n",
                int(ArmorName::not_armor) + 1);
  }

  // ⑥ InvincibleMask 的边界（ROS2 下行用）
  {
    const auto m = InvincibleMask::from_ids({1, 8});
    assert(m.has(ArmorName::one) || m.mask != 0);   // 至少不越界/不崩
    const auto bad = InvincibleMask::from_ids({0, 99, -5});
    assert(bad.mask == 0);   // ⭐ 非法 id 全部忽略（同济的 `ArmorName(id-1)` 会越界）
    std::printf("  [OK] InvincibleMask::from_ids：非法 id(0/99/-5) 全部忽略，不越界\n");

    // ⚠️ 记录：映射表采用同济注释（6=sentry），与 RM 官方协议可能冲突 —— 待与电控核对
    std::printf("  ⚠️  ENEMY_ID_TO_ARMOR: 6->%s, 7->%s（⚠️ 待与电控核对 RM 官方协议）\n",
                ARMOR_NAMES[static_cast<int>(ArmorName::sentry)].c_str(),
                ARMOR_NAMES[static_cast<int>(ArmorName::outpost)].c_str());
  }

  std::printf("\n✅ nav_bridge 全部通过（6 项）\n");
  return 0;
}
