/**
 * @file utils/system/disk_guard.hpp
 * @brief ⭐⭐ **录像磁盘守卫** —— 防止 `records/` 累积写满磁盘
 *
 * ## 为什么需要
 * `Recorder` 每帧写 MJPG，**实测 1.5 MB/s（49 KB/帧 @30fps）**：
 * ```
 *    1 分钟 → 0.09 GB      7 分钟（一场比赛）→ 0.63 GB
 *   60 分钟 → 5.40 GB      ⚠️ 6 小时 → 32.4 GB
 * ```
 * ⚠️ 而 `records/` 的文件**从不自动清理** → 多跑几天就可能写满 SSD。
 *
 * ## 清理策略（⭐ 只在【进程启动时】做一次）
 * | 频率 | 评价 |
 * |---|---|
 * | 每帧 | ⚠️ 扫描目录太贵 |
 * | 每 N 分钟 | ⚠️ 要后台线程，且可能删到**正在写**的文件 |
 * | ⭐ **启动时一次** | ⭐ **最自然**（一次运行 = 一个 session）· **零运行期开销** · **不会删到在写的文件** |
 *
 * ⚠️ 磁盘写满的风险主要来自**累积多次运行** ⇒ **启动时清理正好解决**。
 *
 * ## 做三件事
 * 1. ⭐ **报剩余空间**（让人对"能录多久"有概念）
 * 2. ⭐ **按总大小配额清理最旧的文件**（默认 10 GB ≈ 2 小时录像）
 * 3. ⚠️ **剩余空间过低时明确警告**（不静默）
 */
#ifndef HZMIR_UTILS_SYSTEM_DISK_GUARD_HPP
#define HZMIR_UTILS_SYSTEM_DISK_GUARD_HPP

#include <cstdint>
#include <string>

namespace tools
{

/// 磁盘/目录现状
struct DiskStatus
{
  bool ok = false;             ///< 查询是否成功
  uint64_t dir_bytes = 0;      ///< 目录当前占用
  uint64_t free_bytes = 0;     ///< 所在文件系统剩余
  uint64_t total_bytes = 0;    ///< 所在文件系统总容量
};

/// @brief 查目录占用 + 所属文件系统剩余空间（目录不存在则返回 `ok=false`）
DiskStatus query_disk(const std::string & dir);

/// @brief ⭐ 按**总大小配额**清理目录里最旧的文件（只删常规文件）
/// @param dir       目标目录（如 `"records"`）
/// @param quota_mb  总大小配额（MB）；`0` = 不清理
/// @return 删除的文件数
/// @note ⚠️ **启动时调用一次**即可 —— 见文件头注释的"清理频率"分析。
size_t enforce_quota(const std::string & dir, uint64_t quota_mb);

/// @brief ⭐⭐ 启动时调用：报剩余空间 + 配额清理 + 低空间警告
/// @param dir           录像目录（如 `"records"`）
/// @param quota_mb      总大小配额（MB）；0 = 不清理
/// @param min_free_mb   剩余空间低于此值就**明确警告**（0 = 不检查）
void guard_recording_dir(const std::string & dir, uint64_t quota_mb, uint64_t min_free_mb);

}  // namespace tools

#endif  // HZMIR_UTILS_SYSTEM_DISK_GUARD_HPP
