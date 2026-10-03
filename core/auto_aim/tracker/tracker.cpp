#include "tracker.hpp"

#include <yaml-cpp/yaml.h>

#include <tuple>

#include "utils/log/logger.hpp"
#include "utils/math/math_tools.hpp"

namespace auto_aim
{
Tracker::Tracker(const std::string & config_path, Solver & solver)
: solver_{solver},
  detect_count_(0),
  temp_lost_count_(0),
  state_{"lost"},
  pre_state_{"lost"},
  last_timestamp_(std::chrono::steady_clock::now()),
  omni_target_priority_{ArmorPriority::fifth}
{
  auto yaml = YAML::LoadFile(config_path);
  // The old assignment here targeted a write-only member (removed). Keep the
  // key-presence check: the color filter itself lives in ArmorFilter::apply().
  if (!yaml["enemy_color"]) {
    tools::logger()->warn(
      "[Tracker] yaml 缺 `enemy_color` → 颜色过滤将用 ArmorFilter 的默认值（red）。"
      "请在 params/*.yaml 顶层补上 `enemy_color: \"red\" | \"blue\"`");
  }
  min_detect_count_ = yaml["min_detect_count"].as<int>();
  max_temp_lost_count_ = yaml["max_temp_lost_count"].as<int>();
  outpost_max_temp_lost_count_ = yaml["outpost_max_temp_lost_count"].as<int>();
  normal_temp_lost_count_ = max_temp_lost_count_;

  // ⭐ W7：加载射击过滤器 + 目标优先级（横切能力，原在 omniperception 里）
  filter_.load(yaml);
  // ⭐⭐ 只有在 yaml 里**显式写了** `priority_mode`/`mode` 时才启用优先级
  //   （同济的 tracker 从不设 priority → 默认保持同济行为）
  if (yaml["priority_mode"] || yaml["mode"]) {
    priority_mode_ = parse_priority_mode(yaml, MODE_ONE);
    priority_mode_set_ = true;
    tools::logger()->info("[Tracker] priority_mode = {} （已启用，偏离同济默认）",
                          to_string(priority_mode_));
  } else {
    tools::logger()->debug("[Tracker] 未配置 priority_mode（同济行为）");
  }
}

std::string Tracker::state() const { return state_; }

void Tracker::reload(const YAML::Node & yaml)
{
  filter_.load(yaml);
  priority_mode_ = parse_priority_mode(yaml, priority_mode_);
  tools::logger()->info(
    "[Tracker] 热重载: priority_mode = {}", to_string(priority_mode_));
}

TrackerResult Tracker::track(
  std::list<Armor> & armors, std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // ⭐⭐ W98：本帧进入时的装甲板数（用于算 `filtered_out`）
  const int n_armors_before = static_cast<int>(armors.size());

  // ⭐⭐ W98：**统一出口** —— `track()` 有 4 处早返回（发散/不收敛/丢失/正常），
  //   原来都是裸 `return {};` / `return targets;`。若逐处手写填 dbg，**必然漏**。
  //   ⇒ 收成一个 lambda：所有出口都经它，**结构上不可能忘填**。
  auto finish = [&](std::list<Target> targets) -> TrackerResult {
    TrackerResult r;
    r.targets = std::move(targets);
    r.dbg.state = (state_ == "tracking") ? 2 : (state_ == "temp_lost") ? 3
                  : (state_ == "detecting")            ? 1
                                                       : 0;
    r.dbg.armor_count = static_cast<int>(armors.size());
    r.dbg.priority_mode = static_cast<int>(priority_mode_);
    r.dbg.filtered_out = n_armors_before - static_cast<int>(armors.size());
    // ⚠️ `invincible_count` / `focus_target_count` 由主循环填（它持有 ROS2 原始 id 列表）
    return r;
  };

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }
  // ⭐ W7：统一射击过滤器（①颜色 ②④skip_names ⑤无敌 ⑥集火指令）
  //   原实现只有一句 `a.color != enemy_color_`，其余规则都在 omniperception 里。
  if (use_enemy_color) {
    filter_.apply(armors, invincible_, auto_aim_targets_);
  } else {
    ArmorFilterConfig c = filter_.config();
    c.use_enemy_color = false;
    ArmorFilter(  // 临时构造：跳过颜色检查，其余规则照旧
      c)
      .apply(armors, invincible_, auto_aim_targets_);
  }

  // 过滤前哨站顶部装甲板
  // armors.remove_if([this](const auto_aim::Armor & a) {
  //   return a.name == ArmorName::outpost &&
  //          solver_.oupost_reprojection_error(a, 27.5 * CV_PI / 180.0) <
  //            solver_.oupost_reprojection_error(a, -15 * CV_PI / 180.0);
  // });

  // 优先选择靠近图像中心的装甲板
  // ⭐⭐ W83 修复（同济继承的 bug）：原来硬编码 `img_center(1440/2, 1080/2)`，
  //   ⚠️ 但本仓库相机是 **1280×720** → 中心算成 (720,540)，真实是 (640,360)
  //   ⇒ 「优先选靠近图像中心」的排序**偏向右下角** → 多装甲板时**选错目标**。
  //   （同济原版也是 1440×1080 硬编码 —— 因为他们的相机确实是那个分辨率）
  //   ✅ 改用 `Armor::center_norm`（各 detector 路径都已填，**与分辨率无关**），
  //      中心即归一化坐标的 (0.5, 0.5)。**无需改 `track()` 签名、无需新状态。**
  armors.sort([](const Armor & a, const Armor & b) {
    const cv::Point2f center_norm(0.5f, 0.5f);
    auto distance_1 = cv::norm(a.center_norm - center_norm);
    auto distance_2 = cv::norm(b.center_norm - center_norm);
    return distance_1 < distance_2;
  });

  // ⭐ W7：**仅在显式启用时**打优先级（默认不设 = 同济行为）
  if (priority_mode_set_) set_priority(armors, priority_mode_);

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort(
    [](const auto_aim::Armor & a, const auto_aim::Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return finish({});
  }

  // 收敛效果检测：
  if (
    std::accumulate(
      target_.ekf().recent_nis_failures.begin(), target_.ekf().recent_nis_failures.end(), 0) >=
    (0.4 * target_.ekf().window_size)) {
    tools::logger()->debug("[Target] Bad Converge Found!");
    state_ = "lost";
    return finish({});
  }

  if (state_ == "lost") return finish({});

  return finish({target_});
}

std::tuple<omniperception::DetectionResult, std::list<Target>> Tracker::track(
  const std::vector<omniperception::DetectionResult> & detection_queue, std::list<Armor> & armors,
  std::chrono::steady_clock::time_point t, bool use_enemy_color)
{
  omniperception::DetectionResult switch_target{std::list<Armor>(), t, 0, 0};
  omniperception::DetectionResult temp_target{std::list<Armor>(), t, 0, 0};
  if (!detection_queue.empty()) {
    temp_target = detection_queue.front();
  }

  auto dt = tools::delta_time(t, last_timestamp_);
  last_timestamp_ = t;

  // 时间间隔过长，说明可能发生了相机离线
  if (state_ != "lost" && dt > 0.1) {
    tools::logger()->warn("[Tracker] Large dt: {:.3f}s", dt);
    state_ = "lost";
  }

  // 优先选择靠近图像中心的装甲板
  // ⭐⭐ W83 修复（同济继承的 bug）：原来硬编码 `img_center(1440/2, 1080/2)`，
  //   ⚠️ 但本仓库相机是 **1280×720** → 中心算成 (720,540)，真实是 (640,360)
  //   ⇒ 「优先选靠近图像中心」的排序**偏向右下角** → 多装甲板时**选错目标**。
  //   （同济原版也是 1440×1080 硬编码 —— 因为他们的相机确实是那个分辨率）
  //   ✅ 改用 `Armor::center_norm`（各 detector 路径都已填，**与分辨率无关**），
  //      中心即归一化坐标的 (0.5, 0.5)。**无需改 `track()` 签名、无需新状态。**
  armors.sort([](const Armor & a, const Armor & b) {
    const cv::Point2f center_norm(0.5f, 0.5f);
    auto distance_1 = cv::norm(a.center_norm - center_norm);
    auto distance_2 = cv::norm(b.center_norm - center_norm);
    return distance_1 < distance_2;
  });

  // 按优先级排序，优先级最高在首位(优先级越高数字越小，1的优先级最高)
  armors.sort([](const Armor & a, const Armor & b) { return a.priority < b.priority; });

  bool found;
  if (state_ == "lost") {
    found = set_target(armors, t);
  }

  // 此时主相机画面中出现了优先级更高的装甲板，切换目标
  else if (state_ == "tracking" && !armors.empty() && armors.front().priority < target_.priority) {
    found = set_target(armors, t);
    tools::logger()->debug("auto_aim switch target to {}", ARMOR_NAMES[armors.front().name]);
  }

  // 此时全向感知相机画面中出现了优先级更高的装甲板，切换目标
  else if (
    state_ == "tracking" && !temp_target.armors.empty() &&
    temp_target.armors.front().priority < target_.priority && target_.convergened()) {
    state_ = "switching";
    switch_target = omniperception::DetectionResult{
      temp_target.armors, t, temp_target.delta_yaw, temp_target.delta_pitch};
    omni_target_priority_ = temp_target.armors.front().priority;
    found = false;
    tools::logger()->debug("omniperception find higher priority target");
  }

  else if (state_ == "switching") {
    found = !armors.empty() && armors.front().priority == omni_target_priority_;
  }

  else if (state_ == "detecting" && pre_state_ == "switching") {
    found = set_target(armors, t);
  }

  else {
    found = update_target(armors, t);
  }

  pre_state_ = state_;
  // 更新状态机
  state_machine(found);

  // 发散检测
  if (state_ != "lost" && target_.diverged()) {
    tools::logger()->debug("[Tracker] Target diverged!");
    state_ = "lost";
    return {switch_target, {}};  // 返回switch_target和空的targets
  }

  if (state_ == "lost") return {switch_target, {}};  // 返回switch_target和空的targets

  std::list<Target> targets = {target_};
  return {switch_target, targets};
}

void Tracker::state_machine(bool found)
{
  if (state_ == "lost") {
    if (!found) return;

    state_ = "detecting";
    detect_count_ = 1;
  }

  else if (state_ == "detecting") {
    if (found) {
      detect_count_++;
      if (detect_count_ >= min_detect_count_) state_ = "tracking";
    } else {
      detect_count_ = 0;
      state_ = "lost";
    }
  }

  else if (state_ == "tracking") {
    if (found) return;

    temp_lost_count_ = 1;
    state_ = "temp_lost";
  }

  else if (state_ == "switching") {
    if (found) {
      state_ = "detecting";
    } else {
      temp_lost_count_++;
      if (temp_lost_count_ > 200) state_ = "lost";
    }
  }

  else if (state_ == "temp_lost") {
    if (found) {
      state_ = "tracking";
    } else {
      temp_lost_count_++;
      if (target_.name == ArmorName::outpost)
        //前哨站的temp_lost_count需要设置的大一些
        max_temp_lost_count_ = outpost_max_temp_lost_count_;
      else
        max_temp_lost_count_ = normal_temp_lost_count_;

      if (temp_lost_count_ > max_temp_lost_count_) state_ = "lost";
    }
  }
}

bool Tracker::set_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  if (armors.empty()) return false;

  auto & armor = armors.front();
  solver_.solve(armor);

  // 根据兵种优化初始化参数
  auto is_balance = (armor.type == ArmorType::big) &&
                    (armor.name == ArmorName::three || armor.name == ArmorName::four ||
                     armor.name == ArmorName::five);

  if (is_balance) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 2, P0_dig);
  }

  else if (armor.name == ArmorName::outpost) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 81, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.2765, 3, P0_dig);
  }

  else if (armor.name == ArmorName::base) {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1e-4, 0, 0}};
    target_ = Target(armor, t, 0.3205, 3, P0_dig);
  }

  else {
    Eigen::VectorXd P0_dig{{1, 64, 1, 64, 1, 64, 0.4, 100, 1, 1, 1}};
    target_ = Target(armor, t, 0.2, 4, P0_dig);
  }

  return true;
}

bool Tracker::update_target(std::list<Armor> & armors, std::chrono::steady_clock::time_point t)
{
  target_.predict(t);

  int found_count = 0;
  double min_x = 1e10;  // 画面最左侧
  for (const auto & armor : armors) {
    if (armor.name != target_.name || armor.type != target_.armor_type) continue;
    found_count++;
    min_x = armor.center.x < min_x ? armor.center.x : min_x;
  }

  if (found_count == 0) return false;

  for (auto & armor : armors) {
    if (
      armor.name != target_.name || armor.type != target_.armor_type
      //  || armor.center.x != min_x
    )
      continue;

    solver_.solve(armor);

    target_.update(armor);
  }

  return true;
}

}  // namespace auto_aim