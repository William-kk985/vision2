/**
 * @file utils/system/disk_guard.hpp
 * @brief ⭐⭐ **日志清理 + 磁盘空间报告**
 *
 * ## 为什么需要清理
 * 实测（本仓库开发期）：
 * ```
 *   logs/        236 个文件 / 2.7 MB    ← ⭐ 每次运行写一个（logger.cpp）
 *   output/mvs_log/  1 个文件 / 8 KB     ← 海康 SDK 追加写（W102 起重定向到这里）
 * ```
 * ⚠️ **注意：日志总共才 2.7 MB —— 清它不是为省空间，而是【控制文件数】。**
 * 按每天跑 100 次估算，一年会攒 **~36500 个文件** → 目录没法浏览、`ls` 都卡。
 *
 * ## 清理什么 / 不清理什么
 * | 目录 | 清理 | 理由 |
 * |---|---|---|
 * | ⭐ `logs/`（含 `logs/ros/`） | ✅ **超 30 天删** | 文件数增长快、单文件小 |
 * | ⭐ `output/mvs_log/` | ✅ 超 30 天删 | SDK 日志同样只增不减<br>⚠️ 顶层 `MvSdkLog/`（老版本遗留的真目录）也会兜底清 |
 * | ❌ `records/`（录像） | **不清理** | ⭐ **比赛复盘/取证要用**，且由使用者自行管理 |
 * | ❌ `debug_img/` | **不清理** | 已有上限（默认 500 张），且是主动按 `4` 才产生 |
 *
 * ## ⭐ 清理频率：**只在进程启动时做一次**
 * | 频率 | 评价 |
 * |---|---|
 * | 每帧 | ⚠️ 扫描目录太贵 |
 * | 每 N 分钟 | ⚠️ 要后台线程；且可能删到**正在写**的文件 |
 * | ⭐ **启动时一次** | ⭐ **最自然**（一次运行 = 一个 session）· **零运行期开销** · **不碰正在写的文件** |
 * ⚠️ 文件累积的风险来自**多次运行** ⇒ 启动时清理正好覆盖。
 */
#ifndef HZMIR_UTILS_SYSTEM_DISK_GUARD_HPP
#define HZMIR_UTILS_SYSTEM_DISK_GUARD_HPP

#include <cstdint>
#include <string>
#include <vector>

namespace tools
{

/// 磁盘/目录现状
struct DiskStatus
{
  bool ok = false;             ///< 查询是否成功
  uint64_t dir_bytes = 0;      ///< 目录当前占用
  uint64_t files = 0;          ///< 目录内文件数
  uint64_t free_bytes = 0;     ///< 所在文件系统剩余
  uint64_t total_bytes = 0;    ///< 所在文件系统总容量
};

/// @brief 查目录占用 + 文件数 + 所属文件系统剩余（目录不存在则 `ok=false`）
DiskStatus query_disk(const std::string & dir);

/// @brief ⭐ **删除目录里早于 `days` 天的文件**（按扩展名过滤，递归）
/// @param dir   目标目录（如 `"logs"`）
/// @param days  保留天数（**≤0 = 不清理**，直接返回 0）
/// @param exts  要清理的扩展名（如 `{".log"}`）；**空 = 所有常规文件**
/// @return 删除的文件数
/// @note ⚠️ **启动时调用一次**即可 —— 见文件头注释的"清理频率"分析。
size_t cleanup_older_than(const std::string & dir, int days, const std::vector<std::string> & exts);

/// @brief ⭐⭐ 启动时调用：清理旧日志 + 报告磁盘（**不清理录像**）
/// @param log_days     日志保留天数（默认 30；≤0 = 不清理）
/// @param records_dir  录像目录（**只报告，不清理**）
void guard_on_startup(int log_days, const std::string & records_dir);

}  // namespace tools

#endif  // HZMIR_UTILS_SYSTEM_DISK_GUARD_HPP
