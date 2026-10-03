/**
 * @file utils/log/log_filter.hpp
 * @brief ⭐⭐ W95：**按模块过滤日志** —— 全局级别之外的细粒度控制
 *
 * 背景：全局级别（热键 `d`）只有一档，想「关掉 yolov5 的逐帧刷屏、但保留其它 debug」
 *       做不到。实测 42 个模块共 180 处带标签日志，`yolov5` 这种逐帧日志能把终端刷满。
 *
 * 模块名 = 消息开头的 `[xxx]`（如 `[Gimbal]`、`[yolov5]`、`[Tracker]`）。
 * ⚠️ 判断发生在**格式化之后**，所以动态标签（源码里写 `"[{}] …"`）也能识别。
 * ⚠️ 不以 `[` 开头的消息**一律放行**（不属于任何模块）。
 *
 * 两种模式（互斥，后设置的覆盖前面的）：
 *   · **黑名单**（off）：列出的模块**不打印**，其余照常
 *   · **白名单**（only）：**只**打印列出的模块，其余全丢
 *
 * 用法：
 *   · CLI：`--log-off=yolov5,VirtualBoard` / `--log-only=Tracker,Planner`
 *   · 环境变量：`HZMIR_LOG_OFF=` / `HZMIR_LOG_ONLY=`（同格式，逗号分隔）
 *   · 热键 `n`：循环预设（全部 → 静音噪音 → 只看关键）
 */
#ifndef HZMIR_UTILS_LOG_LOG_FILTER_HPP
#define HZMIR_UTILS_LOG_LOG_FILTER_HPP

#include <string>
#include <vector>

namespace tools
{

/// @brief 解析逗号/空格分隔的模块列表（CLI 与环境变量共用）
std::vector<std::string> parse_module_list(const std::string & s);

/// @brief ⭐ 黑名单：这些模块**不打印**（空 = 清除黑名单）
void set_log_modules_off(const std::vector<std::string> & mods);

/// @brief ⭐ 白名单：**只**打印这些模块（空 = 清除白名单）
void set_log_modules_only(const std::vector<std::string> & mods);

/// @brief 清空两种过滤（= 全部打印）
void clear_log_modules();

/// @brief 当前是否有过滤在生效
bool log_filter_active();

/// @brief 当前状态的可读描述（供热键/启动日志显示）
std::string log_filter_status();

/// @brief ⭐ 是否放行这条（格式化后的）消息 —— 供过滤 sink 调用
bool log_module_should_pass(const char * payload, size_t len);

/// @brief 热键 `n`：循环 全部 → 静音噪音 → 只看关键 → 全部…
/// @return 切换后的状态描述
std::string cycle_log_filter();

}  // namespace tools

#endif  // HZMIR_UTILS_LOG_LOG_FILTER_HPP
