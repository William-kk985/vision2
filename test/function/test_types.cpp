// test/function/test_types.cpp —— 契约层 + utils 冒烟测试（无硬件、无业务）
#include <cassert>
#include <cstdio>
#include <cmath>

#include "core/types.hpp"
#include "utils/math/math_tools.hpp"

int main()
{
  // ═══ ① 契约层：Trajectory（来自 tools/trajectory.hpp）═══
  tools::Trajectory t(22.0, 5.0, 0.5);
  assert(!t.unsolvable);
  std::printf("[OK] tools::Trajectory   pitch=%.4f rad  fly_time=%.4f s\n", t.pitch, t.fly_time);

  // ═══ ② 契约层：Command（来自 io/command.hpp）═══
  io::Command c{};
  c.control = true;
  c.yaw = tools::limit_rad(3.5);
  std::printf("[OK] io::Command         yaw=%.4f (limit_rad)\n", c.yaw);

  // ═══ ③ 契约层：Armor 的枚举 + ⭐ C18 修复验证 ═══
  static_assert(static_cast<int>(auto_aim::ArmorPriority::first) == 1, "priority 顺序");
  auto_aim::Lightbar l, r;
  l.angle = 0.1; l.length = 1.0; l.width = 0.2; l.ratio = 5.0;
  l.center = {0.f, 0.f}; l.top = {0.f, 0.f}; l.bottom = {0.f, 1.f};
  r.angle = 0.1; r.length = 1.0; r.width = 0.2; r.ratio = 5.0;
  r.center = {0.5f, 0.f}; r.top = {0.5f, 0.f}; r.bottom = {0.5f, 1.f};
  auto_aim::Armor a(l, r);
  std::printf("[OK] auto_aim::Armor     priority=%d  (⭐C18 应为 5=fifth)\n",
              static_cast<int>(a.priority));
  assert(a.priority == auto_aim::ArmorPriority::fifth);

  // ═══ ④ ⭐⭐ E2 根因修复：Armor 可默认构造（原来 5 个用户构造 → 无默认构造）═══
  {
    auto_aim::Armor fresh;          // ⭐ 原来这一行**编译不过**
    assert(fresh.priority == auto_aim::ArmorPriority::fifth);
    assert(fresh.confidence == 0.0);
    assert(fresh.duplicated == false);
    assert(fresh.class_id == 0);
    assert(fresh.name == auto_aim::ArmorName::not_armor);
    assert(fresh.xyz_in_world.isZero());
    assert(fresh.ratio == 0.0 && fresh.yaw_raw == 0.0);
    std::printf("[OK] auto_aim::Armor     默认构造可用且成员有确定初值  (⭐E2 已修)\n");

    auto_aim::Lightbar lb;          // ⭐ 原来 `Lightbar() {};` 只声明不初始化 → 垃圾值
    assert(lb.id == 0 && lb.length == 0.0 && lb.angle == 0.0);
    std::printf("[OK] auto_aim::Lightbar  默认构造成员已初始化          (⭐E2 家族)\n");

    io::Command c0{};   // ⭐ E2 家族：Command 的 POD 成员原来全未初始化
    assert(!c0.control && !c0.shoot && c0.yaw == 0.0 && c0.pitch == 0.0);
    io::Command c1;     // ⚠️ 注意：带 NSDMI 后 `Command c;` 也会用默认值
    assert(!c1.control && c1.yaw == 0.0);
    std::printf("[OK] io::Command        默认构造成员已初始化          (⭐E2 家族)\n");
  }

  std::printf("\n✅ 契约层 + utils 全部通过\n");
  return 0;
}
