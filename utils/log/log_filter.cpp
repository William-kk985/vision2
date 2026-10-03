#include "utils/log/log_filter.hpp"

#include <algorithm>
#include <cctype>
#include <mutex>
#include <unordered_set>

namespace tools
{

namespace
{
std::mutex mtx_;
bool only_mode_ = false;                  ///< false = 黑名单模式；true = 白名单模式
std::unordered_set<std::string> mods_;    ///< 当前生效的模块集合

/// @brief 热键 `n` 的预设（依次循环）
struct Preset
{
  const char * name;
  bool only;
  const char * list;
};

// ⭐ 预设依据实测：`yolov5` 是逐帧刷屏大户（每帧 1~2 条），
//   `VirtualBoard` 每 500 条发一次，其余是启动期一次性日志（关掉没意义）。
//   "关键"集合挑选的是**调参时真正要盯的**那几个。
// ⭐ 索引 0 就是程序的初始状态（不过滤）⇒ 第一次按 `n` 会到索引 1（静音噪音），
//   符合直觉：不需要"按一下什么都没变"。
const Preset kPresets[] = {
  {"全部（不过滤）", false, ""},
  {"静音噪音（检测/电控细节）", false,
   "yolov5,YOLOV5,YOLOV8,YOLO11,VirtualBoard,TGD,TI,ReplayCBoard,ReplayBoard,TableTrajectory"},
  {"只看关键（跟踪/规划/射击/云台）", true,
   "Tracker,Planner,Shooter,Gimbal,Target,Priority,Aimer,Solver,ArmorFilter"},
};

size_t preset_idx_ = 0;   ///< ⭐ 记住当前位置，热键循环用

/// ⭐ 内部：只改过滤状态，**不动 preset_idx_** —— 供 cycle_log_filter 用
/// （公开的 set_log_modules_* 会重置循环位置，那是给 CLI/环境变量用的）
void apply_filter(bool only, const std::vector<std::string> & mods)
{
  std::lock_guard<std::mutex> lk(mtx_);
  mods_.clear();
  only_mode_ = only;
  for (const auto & m : mods) if (!m.empty()) mods_.insert(m);
}

std::string lower(std::string s)
{
  std::transform(s.begin(), s.end(), s.begin(),
                 [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
  return s;
}

/// @brief 抠出 `[xxx]` 里的 `xxx`；没有则返回空
std::string extract_module(const char * p, size_t len)
{
  if (len < 3 || p[0] != '[') return {};          // ⚠️ 不以 [ 开头 → 无模块
  for (size_t i = 1; i < len && i < 32; ++i) {    // ⭐ 模块名不会太长，限 32
    if (p[i] == ']') return std::string(p + 1, i - 1);
    if (p[i] == ' ' || p[i] == '\n') return {};   // 空格前还没 ']' → 不是模块标签
  }
  return {};
}
}  // namespace

std::vector<std::string> parse_module_list(const std::string & s)
{
  std::vector<std::string> out;
  std::string cur;
  for (char c : s) {
    if (c == ',' || c == ';' || c == ' ' || c == '\t') {
      if (!cur.empty()) { out.push_back(cur); cur.clear(); }
    } else {
      cur += c;
    }
  }
  if (!cur.empty()) out.push_back(cur);
  return out;
}

void set_log_modules_off(const std::vector<std::string> & mods)
{
  std::lock_guard<std::mutex> lk(mtx_);
  mods_.clear();
  only_mode_ = false;
  for (const auto & m : mods) if (!m.empty()) mods_.insert(m);   // ⭐ 大小写敏感（与源码标签一致）
  preset_idx_ = 0;
}

void set_log_modules_only(const std::vector<std::string> & mods)
{
  std::lock_guard<std::mutex> lk(mtx_);
  mods_.clear();
  only_mode_ = true;
  for (const auto & m : mods) if (!m.empty()) mods_.insert(m);
  preset_idx_ = 0;
}

void clear_log_modules()
{
  std::lock_guard<std::mutex> lk(mtx_);
  mods_.clear();
  only_mode_ = false;
  preset_idx_ = 0;
}

bool log_filter_active()
{
  std::lock_guard<std::mutex> lk(mtx_);
  return !mods_.empty();
}

std::string log_filter_status()
{
  std::lock_guard<std::mutex> lk(mtx_);
  if (mods_.empty()) return "日志过滤: 关（全部打印）";
  std::vector<std::string> v(mods_.begin(), mods_.end());
  std::sort(v.begin(), v.end());
  std::string list;
  for (size_t i = 0; i < v.size(); ++i) list += (i ? "," : "") + v[i];
  return std::string("日志过滤: ") + (only_mode_ ? "只打印 " : "静音 ") + list;
}

bool log_module_should_pass(const char * payload, size_t len)
{
  // ⚠️ 先快速判断有无过滤，避免每条消息都加锁
  {
    std::lock_guard<std::mutex> lk(mtx_);
    if (mods_.empty()) return true;
  }
  const std::string mod = extract_module(payload, len);
  if (mod.empty()) return true;   // ⚠️ 无模块标签 → 一律放行（如启动横幅）

  std::lock_guard<std::mutex> lk(mtx_);
  if (mods_.empty()) return true;
  const bool in = mods_.count(mod) > 0;
  return only_mode_ ? in : !in;   // ⭐ 白名单：在里面才打；黑名单：在里面不打
}

std::string cycle_log_filter()
{
  constexpr size_t kN = sizeof(kPresets) / sizeof(kPresets[0]);
  {
    std::lock_guard<std::mutex> lk(mtx_);
    preset_idx_ = (preset_idx_ + 1) % kN;
  }
  const Preset & p = kPresets[preset_idx_];
  const auto mods = parse_module_list(p.list);
  apply_filter(p.only, mods);   // ⭐ 不经过 set_log_modules_*，避免把循环位置清零
  return std::string(p.name) + "  →  " + log_filter_status();
}

}  // namespace tools
