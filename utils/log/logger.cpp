#include "utils/log/logger.hpp"

#include <fmt/chrono.h>
#include <spdlog/sinks/basic_file_sink.h>
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

void set_logger()
{
  auto file_name = fmt::format("logs/{:%Y-%m-%d_%H-%M-%S}.log", std::chrono::system_clock::now());
  file_sink_ = std::make_shared<spdlog::sinks::basic_file_sink_mt>(file_name, true);
  console_sink_ = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();

  logger_ = std::make_shared<spdlog::logger>(
    "", spdlog::sinks_init_list{file_sink_, console_sink_});
  logger_->flush_on(spdlog::level::info);

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
