#include "utils/system/paths.hpp"

#include <sys/stat.h>
#include <sys/types.h>

#include <cstdlib>
#include <filesystem>

namespace fs = std::filesystem;

namespace tools::paths
{

const char * root()
{
  // ⭐ 可覆盖：方便把输出挂到大盘（`HZMIR_OUTPUT_DIR=/mnt/ssd/hzmir`）
  static const std::string r = [] {
    if (const char * env = std::getenv("HZMIR_OUTPUT_DIR"); env && *env) return std::string(env);
    return std::string("output");
  }();
  return r.c_str();
}

std::string logs() { return std::string(root()) + "/logs"; }
std::string video() { return std::string(root()) + "/video"; }
std::string images() { return std::string(root()) + "/images"; }
std::string csv() { return std::string(root()) + "/csv"; }

// ⭐ W102：海康 SDK 日志目录（SDK 会自己创建，但我们也建一下更稳）
std::string mvs_log() { return std::string(root()) + "/mvs_log"; }

std::string csv_prefix(const std::string & prefix)
{
  if (prefix.empty()) return csv() + "/run";
  // ⭐ 用户显式给了路径（含 `/`）→ 原样尊重，不强制搬到 output/ 下
  if (prefix.find('/') != std::string::npos) return prefix;
  return csv() + "/" + prefix;
}

bool ensure_dir(const std::string & dir)
{
  std::error_code ec;
  if (fs::exists(dir, ec)) return fs::is_directory(dir, ec);
  return fs::create_directories(dir, ec);
}

void ensure_all()
{
  ensure_dir(logs());
  ensure_dir(video());
  ensure_dir(images());
  ensure_dir(csv());
  ensure_dir(mvs_log());

  // ⭐⭐⭐ W102：**把海康 SDK 的 `./MvSdkLog/` 用软链接收进 output/**
  //
  // ⚠️ 为什么不能只靠 `MV_CC_SetSDKLogPath()`：
  //   SDK 在 `MV_CC_EnumDevices()`（**比 CreateHandle 还早**）就会**建好**
  //   `./MvSdkLog/CamCtrl_00.log` 的文件句柄，之后重定向只影响**后续写入** ⇒
  //   顶层仍留一个 **0 字节占位文件**（实测确认）。
  //   ⭐ 把它做成**软链接**指向 `output/mvs_log/` ⇒ 连占位也落在 output 里，
  //     仓库顶层**不再出现** `MvSdkLog/` 实体。
  //
  // ⚠️ 只在我们自己的输出根目录下操作（`HZMIR_OUTPUT_DIR` 改了也成立）；
  //   若已存在**真实目录**（老版本留下的），**不强行删**（可能有日志），只留着并提示。
  {
    namespace fs = std::filesystem;
    const fs::path link = "MvSdkLog";
    const fs::path target = fs::path(root()) / "mvs_log";
    std::error_code ec;
    if (fs::is_symlink(link)) {
      if (fs::read_symlink(link, ec) != target) fs::remove(link, ec), fs::create_symlink(target, link, ec);
    } else if (!fs::exists(link)) {
      fs::create_symlink(target, link, ec);
    }
    // ⚠️ `exists(link)` 且不是 symlink ⇒ 老的真实目录 ⇒ 不动它（只由 disk_guard 清）
  }
}

}  // namespace tools::paths
