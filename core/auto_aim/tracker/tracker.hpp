#ifndef AUTO_AIM__TRACKER_HPP
#define AUTO_AIM__TRACKER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <list>
#include <string>

#include "core/types.hpp"
#include "core/auto_aim/solver/solver.hpp"
#include "core/auto_aim/target/target.hpp"
#include "core/auto_aim/tracker/filter.hpp"
#include "core/auto_aim/tracker/priority.hpp"
#include "utils/concurrency/thread_safe_queue.hpp"

namespace auto_aim
{
class Tracker
{
public:
  Tracker(const std::string & config_path, Solver & solver);

  /// ⭐ W71：暴露 solver（主程序读 `last_debug()` 填 `fd.solver.*`；
  ///   原来 `Tracker` 内部调 `solver_.solve(armor)`，外部完全看不到结果）
  Solver & solver() { return solver_; }
  const Solver & solver() const { return solver_; }

  std::string state() const;

  std::list<Target> track(
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
  Color enemy_color_;
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