#include "utils/system/disk_guard.hpp"

#include <sys/statvfs.h>

#include <algorithm>
#include <filesystem>
#include <map>
#include <vector>

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

  // 目录占用
  for (auto it = fs::recursive_directory_iterator(dir, ec);
       !ec && it != fs::recursive_directory_iterator(); it.increment(ec)) {
    if (it->is_regular_file(ec)) s.dir_bytes += it->file_size(ec);
  }

  // 文件系统剩余（statvfs）
  struct statvfs vfs{};
  if (::statvfs(dir.c_str(), &vfs) == 0) {
    s.free_bytes = static_cast<uint64_t>(vfs.f_bavail) * vfs.f_frsize;
    s.total_bytes = static_cast<uint64_t>(vfs.f_blocks) * vfs.f_frsize;
  }
  s.ok = true;
  return s;
}

size_t enforce_quota(const std::string & dir, uint64_t quota_mb)
{
  if (quota_mb == 0) return 0;
  std::error_code ec;
  if (!fs::exists(dir, ec)) return 0;

  // ⭐ 收集（路径, 大小, mtime）
  struct F { fs::path p; uint64_t sz; fs::file_time_type t; };
  std::vector<F> files;
  uint64_t total = 0;
  for (auto it = fs::directory_iterator(dir, ec);
       !ec && it != fs::directory_iterator(); it.increment(ec)) {
    if (!it->is_regular_file(ec)) continue;
    const auto sz = static_cast<uint64_t>(it->file_size(ec));
    files.push_back({it->path(), sz, it->last_write_time(ec)});
    total += sz;
  }
  const uint64_t quota = quota_mb * kMB;
  if (total <= quota) return 0;

  // ⭐ 按【录像会话】分组：同名 .avi + .txt 是一对（Recorder 的命名规则）
  //   ⚠️ 原来按单个文件删 → 达到配额就 break → 可能留下**孤儿 .txt**（配对 .avi 已删）
  std::map<std::string, std::pair<fs::file_time_type, std::vector<F>>> groups;
  for (const auto & f : files) {
    const auto ext = f.p.string();
    if (ext.size() < 4) continue;
    const auto e4 = ext.substr(ext.size() - 4);
    if (e4 != ".avi" && e4 != ".txt") continue;          // 只碰录像产物
    const auto stem = f.p.stem().string();               // 2026-01-03-00-00-00
    auto & g = groups[stem];
    if (g.second.empty() || f.t < g.first) g.first = f.t;
    g.second.push_back(f);
  }

  // 最旧的会话先删（整组一起）
  std::vector<std::pair<fs::file_time_type, std::string>> order;
  for (const auto & [stem, g] : groups) order.push_back({g.first, stem});
  std::sort(order.begin(), order.end(),
            [](const auto & a, const auto & b) { return a.first < b.first; });

  size_t removed = 0;
  uint64_t freed = 0;
  for (const auto & [t, stem] : order) {
    if (total <= quota) break;
    for (const auto & f : groups[stem].second) {
      std::error_code e2;
      if (fs::remove(f.p, e2)) {
        total -= std::min(total, f.sz);
        freed += f.sz;
        ++removed;
      }
    }
  }
  if (removed)
    tools::logger()->info(
      "[disk] ⭐ `{}` 超过配额 {} → 清理了 {} 个最旧文件（释放 {}）",
      dir, human(quota), removed, human(freed));
  return removed;
}

void guard_recording_dir(const std::string & dir, uint64_t quota_mb, uint64_t min_free_mb)
{
  // ⭐ 配额清理（启动时一次）
  enforce_quota(dir, quota_mb);

  const auto s = query_disk(dir);
  if (!s.ok) {
    tools::logger()->debug("[disk] `{}` 不存在（还没录过像）", dir);
    return;
  }

  // ⭐ 报"能录多久"（1.5 MB/s 实测）
  const uint64_t rate = static_cast<uint64_t>(1.5 * kMB);   // 1.5 MB/s
  const uint64_t can_rec_s = rate ? s.free_bytes / rate : 0;

  tools::logger()->info(
    "[disk] `{}` 占用 {} · 分区剩余 {} / {}（按实测 1.5 MB/s 估算 **可录约 {} 分钟**）",
    dir, human(s.dir_bytes), human(s.free_bytes), human(s.total_bytes), can_rec_s / 60);

  // ⚠️ 低空间警告（不静默）
  if (min_free_mb && s.free_bytes < min_free_mb * kMB) {
    tools::logger()->warn(
      "[disk] ⚠️⚠️ 剩余空间仅 {}（低于阈值 {}）→ **录像可能写满磁盘**。\n"
      "    可选：① 清理 `{}`（本程序启动时会按配额自动清理最旧文件）\n"
      "          ② 用 `--record=false` 不录像（其余 Debug 通道不受影响）\n"
      "          ③ 把录像目录挂到大容量分区",
      human(s.free_bytes), human(min_free_mb * kMB), dir);
  }
}

}  // namespace tools
