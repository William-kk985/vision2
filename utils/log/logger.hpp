/**
 * @file utils/log/logger.hpp
 * @brief 日志器（⭐ W18：加**运行期级别切换**）
 *
 * doc 09 §14.3 第 ③ 层：「运行期 logger 级别」——
 * 赛场上把级别调到 `warn` 可以省掉大量格式化 + IO 成本（debug 日志实测能刷满终端）。
 *
 * 用法：
 *   · 环境变量 `HZMIR_LOG_LEVEL=warn` 设定启动级别
 *   · 运行期 `cycle_log_level()` 循环切换（绑定到热键 `d`）
 */
#ifndef HZMIR_UTILS_LOG_LOGGER_HPP
#define HZMIR_UTILS_LOG_LOGGER_HPP

#include <spdlog/spdlog.h>

#include <memory>
#include <string>

namespace tools
{

std::shared_ptr<spdlog::logger> logger();

/// @brief 设置运行期级别（同时作用于 console 与 file sink）
void set_log_level(spdlog::level::level_enum lv);

/// @brief 当前级别
spdlog::level::level_enum log_level();

/// @brief 循环切换 debug → info → warn → error → off → debug（供热键 `d`）
/// @return 切换后的级别
spdlog::level::level_enum cycle_log_level();

/// @brief 级别的短名（"debug" / "info" / ...）
const char * log_level_name(spdlog::level::level_enum lv);

/// @brief 解析 "debug"/"info"/"warn"/"error"/"off"（大小写不敏感）
/// @return 是否解析成功
bool parse_log_level(const std::string & s, spdlog::level::level_enum & out);

}  // namespace tools

#endif  // HZMIR_UTILS_LOG_LOGGER_HPP
