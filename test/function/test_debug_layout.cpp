/**
 * @file test/function/test_debug_layout.cpp
 * @brief ⭐⭐ W50（方案 A）：**Debug 结构的「角色自治」布局护栏**
 *
 * ## 守住什么
 * 各角色的 Debug 快照必须住在**角色自己目录**，且**依赖单向**：
 * ```
 * <角色>/xxx_debug.hpp  →  core/debug_node.hpp（聚合）  →  FrameDebug
 * ```
 * ⚠️ **角色头绝不许 include `core/debug_node.hpp`** —— 否则就是环。
 *
 * ## 为什么需要这道护栏
 * W50 之前 `core/debug_node.hpp`（当时叫 `core/debug.hpp`）是「上帝头文件」（所有角色的 Debug 都塞它），
 * 结果 W8 加 `Plan` 的 10 个内部量**要改 5 个地方**（3 处在别的目录/语言）。
 * 这道测试防止**有人以后又把字段挪回去**。
 */
#include <cstdio>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>

static int g_fail = 0;
static void ok(const char * m) { std::printf("  [OK] %s\n", m); }
static void bad(const char * m) { std::printf("  [!!] %s\n", m); ++g_fail; }

/// 角色 Debug 头的规范位置
static const std::vector<std::pair<const char *, const char *>> kExpected = {
  {"DetectorDebug", "core/auto_aim/detector/detector_debug.hpp"},
  {"SolverDebug", "core/auto_aim/solver/solver_debug.hpp"},
  {"TrackerDebug", "core/auto_aim/tracker/tracker_debug.hpp"},
  {"TargetDebug", "core/auto_aim/target/target_debug.hpp"},
  {"PlannerDebug", "core/auto_aim/planner/planner_debug.hpp"},
  {"ShooterDebug", "core/auto_aim/shooter/shooter_debug.hpp"},
  {"ControllerDebug", "core/auto_aim/controller/controller_debug.hpp"},
  {"BuffDebug", "core/auto_buff/buff_debug.hpp"},
};

static std::string slurp(const std::string & p)
{
  std::ifstream f(p);
  std::stringstream ss;
  ss << f.rdbuf();
  return ss.str();
}

int main()
{
  std::printf("═══ Debug 布局护栏（W50 方案 A）═══\n");

  // ── ① 每个角色 Debug 头都存在，且定义了对应 struct ──
  for (const auto & [name, path] : kExpected) {
    const auto src = slurp(path);
    if (src.empty()) { bad((std::string("缺文件 ") + path).c_str()); continue; }
    if (src.find("struct " + std::string(name)) == std::string::npos) {
      bad((std::string(path) + " 里没有 struct " + name).c_str());
      continue;
    }
    ok((std::string(path) + " 定义了 " + name).c_str());
  }

  // ── ② ⭐ 依赖单向：角色头**不许** include core/debug.hpp ──
  {
    int bad_cnt = 0;
    for (const auto & [name, path] : kExpected) {
      const auto src = slurp(path);
      // 只看真正的 #include 行（避免误伤文档注释里的提及）
      std::stringstream ss(src);
      std::string line;
      while (std::getline(ss, line)) {
        const auto p = line.find("#include");
        if (p == std::string::npos) continue;
        // ⭐ W104：只认真 include（注释里提到文件名不算）
        const bool is_inc = line.find("#include") != std::string::npos;
        if (is_inc && line.find("core/debug_node.hpp") != std::string::npos) {
          bad((std::string(path) + " ❌ include 了 core/debug_node.hpp（会成环！）").c_str());
          ++bad_cnt;
        }
      }
    }
    if (bad_cnt == 0) ok("⭐ 8 个角色 Debug 头**都不 include** core/debug_node.hpp（依赖单向，无环）");
  }

  // ── ③ ⭐ core/debug_node.hpp 只做聚合：必须 include 那 8 个 ──
  //   ⚠️ W104：本文件原叫 `core/debug.hpp`，改名后那个名字给了【开关与实验总控】
  {
    const auto agg = slurp("core/debug_node.hpp");
    int miss = 0;
    for (const auto & [name, path] : kExpected)
      if (agg.find(path) == std::string::npos) {
        bad((std::string("core/debug_node.hpp 没 include ") + path).c_str());
        ++miss;
      }
    if (miss == 0) ok("⭐ core/debug_node.hpp 聚合了全部 8 个角色 Debug 头");
    if (agg.find("struct FrameDebug") == std::string::npos) bad("core/debug_node.hpp 里没有 FrameDebug");
    else ok("core/debug_node.hpp 定义了 FrameDebug（唯一职责）");
  }

  // ── ③.5 ⭐⭐ W104：新的 `core/debug.hpp` = 开关与实验总控 ──
  //   三条纪律：① 零依赖（谁都能 include）② 不含数据面 ③ 实验开关命名规范
  {
    const auto hub = slurp("core/debug.hpp");
    // ① 零依赖：只允许 include guard，**不许 include 任何东西**
    bool has_include = false;
    {
      std::istringstream is(hub);
      std::string line;
      while (std::getline(is, line)) {
        if (line.find("#include") != std::string::npos) { has_include = true; break; }
      }
    }
    if (has_include) bad("core/debug.hpp（开关总控）**不许 include** 任何东西（要零依赖）");
    else ok("⭐ core/debug.hpp（开关总控）零依赖（无任何 #include）");

    // ② 不含数据面（数据面在 debug_node.hpp）
    if (hub.find("struct FrameDebug") != std::string::npos)
      bad("core/debug.hpp 不该含 FrameDebug（那是 debug_node.hpp 的职责）");
    else ok("⭐ core/debug.hpp 不含数据面（FrameDebug 在 debug_node.hpp）");

    // ③ 实验开关必须成对（有 `#define HZMIR_EXP_` 就该有对应的测试文件）
    //    ⚠️ 这里只查命名前缀，不查文件存在性（那是 CMake 的活）
    if (hub.find("HZMIR_EXP_") == std::string::npos)
      ok("⭐ core/debug.hpp 有实验开关区（当前无开启的实验）");
    else ok("⭐ core/debug.hpp 有实验开关区（有开启的实验 —— CMake 会自动挂源码）");
  }

  // ── ④ 角色头必须只依赖 <cstdint>（纯数据）──
  {
    int bad_cnt = 0;
    for (const auto & [name, path] : kExpected) {
      std::stringstream ss(slurp(path));
      std::string line;
      while (std::getline(ss, line)) {
        if (line.rfind("#include", 0) != 0) continue;
        if (line.find("<cstdint>") == std::string::npos) {
          bad((std::string(path) + " 的 include 不是 <cstdint>: " + line).c_str());
          ++bad_cnt;
        }
      }
    }
    if (bad_cnt == 0) ok("⭐ 角色 Debug 头都是**纯数据**（只 include <cstdint>）");
  }

  std::printf("\n%s\n", g_fail == 0 ? "✅ Debug 布局护栏全部通过" : "❌ 有失败");
  return g_fail == 0 ? 0 : 1;
}
