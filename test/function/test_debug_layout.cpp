/**
 * @file test/function/test_debug_layout.cpp
 * @brief ⭐⭐ W50（方案 A）：**Debug 结构的「角色自治」布局护栏**
 *
 * ## 守住什么
 * 各角色的 Debug 快照必须住在**角色自己目录**，且**依赖单向**：
 * ```
 * <角色>/xxx_debug.hpp  →  core/debug.hpp（聚合）  →  FrameDebug
 * ```
 * ⚠️ **角色头绝不许 include `core/debug.hpp`** —— 否则就是环。
 *
 * ## 为什么需要这道护栏
 * W50 之前 `core/debug.hpp` 是「上帝头文件」（所有角色的 Debug 都塞它），
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
        if (line.find("core/debug.hpp") != std::string::npos) {
          bad((std::string(path) + " ❌ include 了 core/debug.hpp（会成环！）").c_str());
          ++bad_cnt;
        }
      }
    }
    if (bad_cnt == 0) ok("⭐ 8 个角色 Debug 头**都不 include** core/debug.hpp（依赖单向，无环）");
  }

  // ── ③ ⭐ core/debug.hpp 只做聚合：必须 include 那 8 个 ──
  {
    const auto agg = slurp("core/debug.hpp");
    int miss = 0;
    for (const auto & [name, path] : kExpected)
      if (agg.find(path) == std::string::npos) {
        bad((std::string("core/debug.hpp 没 include ") + path).c_str());
        ++miss;
      }
    if (miss == 0) ok("⭐ core/debug.hpp 聚合了全部 8 个角色 Debug 头");
    if (agg.find("struct FrameDebug") == std::string::npos) bad("core/debug.hpp 里没有 FrameDebug");
    else ok("core/debug.hpp 定义了 FrameDebug（唯一职责）");
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
