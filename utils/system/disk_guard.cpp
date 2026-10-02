#include "utils/system/disk_guard.hpp"

#include <sys/statvfs.h>

#include <algorithm>
#include <chrono>
#include <filesystem>

#include "utils/log/logger.hpp"

namespace fs = std::filesystem;

namespace tools
{

namespace
{
constexpr uint64_t kMB = 1024ull * 1024ull;
std::string human(uint64_t b)
{
  if (b >= 1024ull * kMB) return std::to_string(b / (1024 * kMB)) + " GB";
  if (b >= kMB) return std::to_string(b / kMB) + " MB";
  return std::to_string(b / 1024) + " KB";
}
}  // namespace

DiskStatus query_disk(const std::string & dir)
{
  DiskStatus s;
  std::error_code ec;
  if (!fs::exists(dir, ec)) return s;

  for (auto it = fs::recursive_directory_iterator(dir, ec);
       !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
    if (it->is_regular_file(ec)) {
      s.dir_bytes += it->file_size(ec);
      ++s.files;
    }
  }

  struct statvfs vfs{};
  if (::statvfs(dir.c_str(), &vfs) == 0) {
    s.free_bytes = static_cast<uint64_t>(vfs.f_bavail) * vfs.f_frsize;
    s.total_bytes = static_cast<uint64_t>(vfs.f_blocks) * vfs.f_frsize;
  }
  s.ok = true;
  return s;
}

size_t cleanup_older_than(const std::string & dir, int days, const std::vector<std::string> & exts)
{
  if (days <= 0) return 0;                       // ⭐ ≤0 = 不清理
  std::error_code ec;
  if (!fs::exists(dir, ec)) return 0;

  // ⭐ 以"现在 - days 天"为界（用 file_time_type，避免 system_clock/steady_clock 混用）
  const auto now_ft = fs::file_time_type::clock::now();
  const auto cutoff = now_ft - std::chrono::hours(24ll * days);

  auto ext_ok = [&](const fs::path & p) {
    if (exts.empty()) return true;              // 空 = 所有常规文件
    const auto e = p.extension().string();
    return std::find(exts.begin(), exts.end(), e) != exts.end();
  };

  size_t removed = 0;
  uint64_t freed = 0;

  // ⭐ 递归（`logs/ros/` 里 ROS 会建日期子目录）
  for (auto it = fs::recursive_directory_iterator(
         dir, fs::directory_options::skip_permission_denied, ec);
       !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
    std::error_code e2;
    if (!it->is_regular_file(e2)) continue;
    if (!ext_ok(it->path())) continue;
    const auto t = it->last_write_time(e2);
    if (e2 || t >= cutoff) continue;            // 够新 → 保留
    const auto sz = it->file_size(e2);
    if (fs::remove(it->path(), e2)) { ++removed; freed += sz; }
  }

  if (removed)
    tools::logger()->info(
      "[disk] `{}` 清理了 {} 个超过 {} 天的文件（释放 {}）", dir, removed, days, human(freed));
  return removed;
}

void guard_on_startup(int log_days, const std::string & records_dir)
{
  // ── ① ⭐ 清理旧日志（**启动时一次**）──
  //   ⚠️ 实测 logs/ 236 个文件才 2.7 MB —— 清它是为**控制文件数**（目录可浏览），不为省空间。
  cleanup_older_than("logs", log_days, {".log"});
  cleanup_older_than("MvSdkLog", log_days, {".log"});   // 海康 SDK 的日志（只增不减）

  // ── ② 报告现状（**只报告，不清理录像**）──
  const auto lg = query_disk("logs");
  if (lg.ok)
    tools::logger()->debug(
      "[disk] logs/ {} 个文件 {}（清理阈值 {} 天）", lg.files, human(lg.dir_bytes), log_days);

  // ⭐ 录像：**只报"能录多久"，不清理**（比赛复盘/取证要用，交使用者自行管理）
  const auto rc = query_disk(records_dir);
  if (rc.ok && rc.free_bytes) {
    constexpr uint64_t kRate = static_cast<uint64_t>(1.5 * 1024 * 1024);   // 实测 1.5 MB/s
    tools::logger()->debug(
      "[disk] {}/ {} 个文件 {} · 分区剩余 {} · 可录约 {} 分钟（1.5 MB/s）· 不自动清理",
      records_dir, rc.files, human(rc.dir_bytes), human(rc.free_bytes), rc.free_bytes / kRate / 60);
  }
}

}  // namespace tools
