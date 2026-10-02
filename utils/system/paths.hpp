/**
 * @file utils/system/paths.hpp
 * @brief ⭐⭐ **所有运行期输出路径的【单一真相源】**
 *
 * ## 为什么需要
 * 原来输出散在 4 个不相干的位置，根目录会攒下 CSV：
 * ```
 *   logs/            日志          （logger.cpp）
 *   records/         录像 .avi/.txt（recorder.cpp）
 *   debug_imgs/      存图 .png     （debug_setup.hpp）
 *   <prefix>_frames.csv   ⚠️ 直接落在【项目根目录】（csv_sink.cpp）
 * ```
 * ⇒ 现在统一到 `output/` 下，每个用途一个子目录：
 * ```
 *   output/
 *   ├── logs/     日志（含 ros/ 子目录）
 *   ├── video/    录像（.avi + 同名 .txt 位姿）
 *   ├── images/   存图（.png）
 *   └── csv/      CSV（<prefix>_frames.csv / _series.csv）
 * ```
 * ⚠️ **海康 SDK 的 `MvSdkLog/` 不在本表** —— 那是厂商 SDK 自己决定的路径
 *   （可用 `MV_CC_SetSDKLogPath` 改，但没必要动它）。
 *
 * ## 用法
 * ```cpp
 * logger()->info("-> {}", tools::paths::csv_file("run1", "frames"));
 * ```
 * ⭐ **不要在任何地方再写裸字符串 `"logs/"`** —— 都从这里取。
 */
#ifndef HZMIR_UTILS_SYSTEM_PATHS_HPP
#define HZMIR_UTILS_SYSTEM_PATHS_HPP

#include <string>

namespace tools::paths
{

/// 所有输出的根目录（可用环境变量 `HZMIR_OUTPUT_DIR` 覆盖）
const char * root();

/// 日志目录（`output/logs`）
std::string logs();

/// 录像目录（`output/video`）
std::string video();

/// 存图目录（`output/images`）
std::string images();

/// CSV 目录（`output/csv`）
std::string csv();

/// ⭐ 解析 CSV 前缀：若已含 `/` 就**原样使用**（尊重显式路径），否则放到 `csv()` 下
/// @param prefix 用户给的 `--csv=` 值（如 `run1` 或 `/tmp/x/run1`）
std::string csv_prefix(const std::string & prefix);

/// 确保目录存在（递归创建；已存在返回 true）
bool ensure_dir(const std::string & dir);

/// ⭐ 启动时一次性建好所有输出目录（幂等）
void ensure_all();

}  // namespace tools::paths

#endif  // HZMIR_UTILS_SYSTEM_PATHS_HPP
