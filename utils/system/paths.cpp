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
}

}  // namespace tools::paths
