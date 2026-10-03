#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "core/types.hpp"
#include "core/auto_aim/solver/solver.hpp"
#include "core/auto_aim/solver/solver_debug.hpp"   // ⭐ W101
#include "core/auto_aim/tracker/tracker_debug.hpp"   // ⭐ W98
#include "core/auto_aim/target/target.hpp"
#include "core/auto_aim/tracker/filter.hpp"
#include "core/auto_aim/tracker/priority.hpp"
#include "utils/concurrency/thread_safe_queue.hpp"

namespace auto_aim
{

/// ⭐⭐ W98：`track()` 的返回值 —— **目标 + 调试快照一起返回**
///
/// 放在这里而不是 `tracker_debug.hpp`：那个头是**零依赖 POD**（硬纪律），
/// 而本结构含业务类型 `Target`。
struct TrackerResult
{
  std::list<Target> targets;
  TrackerDebug dbg;

  // ⭐⭐⭐ W101（原 F8）：**顺手把 Solver 的调试快照也带出来**。
  //
  // ⚠️ 原来 `Solver::solve()` 是 `const`，把调试量存进 `mutable last_dbg_`，
  //   外部靠 `tracker.solver().last_debug()` **主动去取** —— 那是**半个 Result 模式**
  //   （数据是出来了，但要人记得取；忘了就静默为默认值）。
  // ⭐ 而 `solve()` 是在 **`Tracker::track()` 内部**被调用的 ⇒ **Tracker 最清楚结果**
  //   ⇒ 让它带出来，主循环一行 `fd.solver = trk.solver_dbg;`。
  //   `SolverDebug` 是零依赖 POD，放这里不违反任何纪律。
  SolverDebug solver_dbg;
};

class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  /// ⭐ W71：暴露 solver（主程序读 `last_debug()` 填 `fd.solver.*`；
  ///   原来 `Tracker` 内部调 `solver_.solve(armor)`，外部完全看不到结果）
  Solver & solver() { return solver_; }
  const Solver & solver() const { return solver_; }

  std::string state() const;

  /// ⭐⭐ W98：返回 `TrackerResult`（目标 + 调试快照）
  ///
  /// ⚠️ `TrackerResult` 定义在**本文件**（而非 `tracker_debug.hpp`）——
  ///   因为 `tracker_debug.hpp` 是**零依赖 POD**（只 include `<cstdint>`），
  ///   而 `TrackerResult` 含 `std::list<Target>`（业务类型）。
  ///   这条纪律见 `detector_debug.hpp` 的长注释：**POD 与"带业务类型的 Result"分开**。
  ///
  /// ## 为什么（同 Detector，见 `detector.hpp` 的长注释）
  /// 原来主循环要**手写 6 行**从 tracker 各处取值填 `fd.tracker`，还容易漏。
  /// 现在 tracker **在自己的统一出口填好**，主循环只要 `fd.tracker = r.dbg;`。
  /// ⚠️ 例外：`invincible_count` / `focus_target_count` 来自 ROS2 下行数据，
  ///   主循环知道得更准（它持有原始 id 列表）→ 由主循环在那之后再覆盖。
  TrackerResult track(
    std::list<Armor> & armors, std::chrono::steady_clock::time_point t,
    bool use_enemy_color = true);

  // ⭐ W7 新增：横切能力接口（从 omniperception 捞出）
  /// @brief 设置无敌掩码（来自下位机/ROS 的比赛信息）
  void set_invincible(const InvincibleMask & m) { invincible_ = m; }

  /// ⭐ W72：读无敌掩码（主程序填 `fd.target.invincible`；原来该列永远是 0）
  const InvincibleMask & invincible() const { return invincible_; }
  /// @brief 设置集火指令（上级指定的优先目标；空 = 不限制）
  void set_auto_aim_targets(std::vector<ArmorName> t) { auto_aim_targets_ = std::move(t); }
  /// @brief 运行时切换优先级模式（⭐ 开启后即偏离同济行为）
  void set_priority_mode(PriorityMode m)
  {
    priority_mode_ = m;
    priority_mode_set_ = true;
  }
  PriorityMode priority_mode() const { return priority_mode_; }

  /// ⭐ W21：热重载（射击过滤器 + 优先级模式）—— 按键 `r` 用
  /// ⚠️ 只重载**安全项**：不改检测/跟踪的状态机参数（那会让正在跟踪的目标抖）
  void reload(const YAML::Node & yaml);
  const ArmorFilter & filter() const { return filter_; }

  std::tuple<omniperception::DetectionResult, std::list<Target>> track(
    const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
    std::chrono::steady_clock::time_point t, bool use_enemy_color = true);

private:
  Solver & solver_;
  // ⚠️ W82 删除：`Color enemy_color_;` —— **只赋值、从不读取**的死成员。
  //   颜色过滤实际发生在 `ArmorFilter::apply()`（它自己从 yaml 读 `enemy_color`，
  //   见 W69 修复：顶层为准 + `armor_filter` 段可覆盖）。
  //   历史上这里还有个副作用：构造函数里 `yaml["enemy_color"].as<std::string>()`
  //   在缺键时会抛异常（当校验用）→ 已改为显式校验（见 .cpp）。
  int min_detect_count_;
  int max_temp_lost_count_;
  int detect_count_;
  int temp_lost_count_;
  int outpost_max_temp_lost_count_;
  int normal_temp_lost_count_;
  std::string state_, pre_state_;
  Target target_;
  std::chrono::steady_clock::time_point last_timestamp_;
  ArmorPriority omni_target_priority_;
  // ⭐ W7：从 omniperception 捞出的横切能力
  ArmorFilter filter_;                        // 射击过滤器（5 条规则）
  // ⭐⭐ 默认**不设优先级** = 同济行为
  //   同济的 `tracker` 从不给 `armor.priority` 赋值，却用它排序 —— 那是 C18（未初始化 UB）。
  //   我们已把默认值固定为 `fifth`（消除 UB，见 core/types.hpp），但**排序行为仍与同济一致**
  //   （全是 fifth → 等价于不排序）。开启优先级模式才启用 4 张表。
  PriorityMode priority_mode_ = MODE_ONE;
  bool priority_mode_set_ = false;
  InvincibleMask invincible_;                 // 无敌掩码
  std::vector<ArmorName> auto_aim_targets_;   // 集火指令

  void state_machine(bool found);

  bool set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);

  bool update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t);
};

}  // namespace auto_aim

#endif  // AUTO_AIM__TRACKER_HPP