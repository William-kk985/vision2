#include "utils/log/logger.hpp"

#include "utils/system/paths.hpp"   // W87: unified output/ layout
#include <fmt/chrono.h>
#include "utils/log/log_filter.hpp"
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/dist_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <algorithm>
#include <cstdlib>
#include <cctype>
#include <chrono>
#include <string>

namespace tools
{

namespace
{
std::shared_ptr<spdlog::logger> logger_ = nullptr;
// ⭐ W18：保留两个 sink 的句柄，便于运行期改级别（原来 sink 是局部的，改不了）
std::shared_ptr<spdlog::sinks::basic_file_sink_mt> file_sink_;
std::shared_ptr<spdlog::sinks::stdout_color_sink_mt> console_sink_;

constexpr spdlog::level::level_enum DEFAULT_LEVEL = spdlog::level::debug;

/// ⭐⭐ W95：**按模块过滤的 sink** —— 包在 console/file 外面，先判模块再转发。
///
/// 为什么用 `dist_sink`：它本身是个"分发器"（可挂多个子 sink），
/// 只要重写 `sink_it_` 就能在转发前插一层过滤，且**子 sink 的级别设置不受影响**。
/// ⚠️ 过滤发生在 `log_msg` 已 format 之后？—— 不是：`log_msg.payload` 就是
///    **格式化好的文本**（spdlog 在 `logger::log_it_` 里先 format 再交给 sink），
///    所以动态标签（`"[{}] ..."`）也能被正确识别。
class ModuleFilterSink : public spdlog::sinks::dist_sink<std::mutex>
{
protected:
  void sink_it_(const spdlog::details::log_msg & msg) override
  {
    if (!tools::log_module_should_pass(msg.payload.data(), msg.payload.size())) return;  // ⭐ 丢弃
    spdlog::sinks::dist_sink<std::mutex>::sink_it_(msg);
  }
  void flush_() override { spdlog::sinks::dist_sink<std::mutex>::flush_(); }
};

std::shared_ptr<ModuleFilterSink> filter_sink_;   // ⭐ 保留句柄（虽然运行期只改内部状态）

void set_logger()
{
  // ⭐ W87：输出统一到 `output/`（见 utils/system/paths.hpp）
  tools::paths::ensure_all();
  auto file_name = fmt::format("{}/{:%Y-%m-%d_%H-%M-%S}.log", tools::paths::logs(),
                               std::chrono::system_clock::now());
  file_sink_ = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, true);
  console_sink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

  // ⭐⭐ W95：链式结构 logger → ModuleFilterSink → {file_sink, console_sink}
  filter_sink_ = std::make_shared<ModuleFilterSink>();
  filter_sink_->add_sink(file_sink_);
  filter_sink_->add_sink(console_sink_);

  logger_ = std::make_shared<spdlog::logger>("", filter_sink_);
  logger_->flush_on(spdlog::level::info);

  // ⭐ 启动时的模块过滤：环境变量（CLI 在 main 里覆盖）
  if (const char * off = std::getenv("HZMIR_LOG_OFF"); off && *off)
    set_log_modules_off(parse_module_list(off));
  if (const char * only = std::getenv("HZMIR_LOG_ONLY"); only && *only)
    set_log_modules_only(parse_module_list(only));

  // ⭐ 启动级别：环境变量 > 默认
  spdlog::level::level_enum lv = DEFAULT_LEVEL;
  if (const char * env = std::getenv("HZMIR_LOG_LEVEL"); env && *env) {
    spdlog::level::level_enum parsed;
    if (parse_log_level(env, parsed)) lv = parsed;
    else
      logger_->warn("HZMIR_LOG_LEVEL 无法识别: {}（用默认）", env);
  }
  set_log_level(lv);
}
}  // namespace

std::shared_ptr<spdlog::logger> logger()
{
  if (!logger_) set_logger();
  return logger_;
}

void set_log_level(spdlog::level::level_enum lv)
{
  auto lg = logger();
  lg->set_level(lv);
  if (filter_sink_) filter_sink_->set_level(lv);   // ⭐ W95
  if (console_sink_) console_sink_->set_level(lv);
  if (file_sink_) file_sink_->set_level(lv);
}

spdlog::level::level_enum log_level() { return logger()->level(); }

spdlog::level::level_enum cycle_log_level()
{
  using L = spdlog::level::level_enum;
  static const L order[] = {L::debug, L::info, L::warn, L::err, L::off};   // ⚠️ spdlog 用 err 不是 error
  const L cur = log_level();
  size_t i = 0;
  for (size_t k = 0; k < sizeof(order) / sizeof(order[0]); ++k)
    if (order[k] == cur) { i = k; break; }
  const L next = order[(i + 1) % (sizeof(order) / sizeof(order[0]))];
  set_log_level(next);
  return next;
}

const char * log_level_name(spdlog::level::level_enum lv)
{
  switch (lv) {
    case spdlog::level::trace:    return "trace";
    case spdlog::level::debug:    return "debug";
    case spdlog::level::info:     return "info";
    case spdlog::level::warn:     return "warn";
    case spdlog::level::err:      return "error";
    case spdlog::level::critical: return "critical";
    case spdlog::level::off:      return "off";
    default:                      return "?";
  }
}

bool parse_log_level(const std::string & s, spdlog::level::level_enum & out)
{
  std::string t;
  t.reserve(s.size());
  for (char c : s) t.push_back(static_cast<char>(std::tolower(static_cast<unsigned char>(c))));

  if (t == "trace")    { out = spdlog::level::trace; return true; }
  if (t == "debug")    { out = spdlog::level::debug; return true; }
  if (t == "info")     { out = spdlog::level::info; return true; }
  if (t == "warn" || t == "warning") { out = spdlog::level::warn; return true; }
  if (t == "error" || t == "err")    { out = spdlog::level::err; return true; }
  if (t == "critical") { out = spdlog::level::critical; return true; }
  if (t == "off" || t == "none")     { out = spdlog::level::off; return true; }
  return false;
}

}  // namespace tools
