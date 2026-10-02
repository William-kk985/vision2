#include "utils/system/host_info.hpp"

#include <cstdlib>
#include <thread>

#include "utils/log/logger.hpp"

namespace tools
{

/// ⭐ 各 sink 的实测开销（自瞄线程 + 后台），**只列事实**
namespace
{
constexpr const char * kCostTable =
  "      · 窗口    (按键 1)      ms 级, 在自瞄线程【阻塞】          —— 开销最大\n"
  "      · 录像    (--record)    ~1.0 核   (MJPG 编码在 worker)\n"
  "      · 存图    (按键 4)      ~0.15 核  (PNG 编码在 worker; 自瞄线程 33 us 深拷贝)\n"
  "      · CSV     (按键 2)      ~0.01 核  (自瞄线程 250 ns)\n"
  "      · PlotJuggler (按键 3)  ~0.01 核  (自瞄线程 104 ns)\n"
  "      · 不挂任何 sink         ~0        (仅 L0/L1 常驻约 400 ns)";
}  // namespace

HostInfo detect_host()
{
  HostInfo h;
  h.logical_cores = std::thread::hardware_concurrency();
  if (h.logical_cores == 0) h.logical_cores = 1;   // 探测失败 → 按最保守算

  // 覆盖：`HZMIR_CORES=<n>` —— 用于 ① 测试各档位 ② 容器/虚拟机里被限核时手动指定
  //   （`hardware_concurrency()` 返回【整机】逻辑核，不受 taskset/cgroup 影响）
  if (const char * env = std::getenv("HZMIR_CORES"); env && *env) {
    const int v = std::atoi(env);
    if (v > 0) {
      h.logical_cores = static_cast<unsigned>(v);
      tools::logger()->info("[host] 核数被 HZMIR_CORES 覆盖为 {}", v);
    }
  }

  // ⭐ 档位只影响「提醒语气」，**不代替使用者判断**
  const unsigned n = h.logical_cores;
  if (n >= 16) {
    h.tier = HostTier::Roomy;
    h.tier_name = "充裕";
  } else if (n >= 8) {
    h.tier = HostTier::Okay;
    h.tier_name = "正常";
  } else if (n >= 4) {
    h.tier = HostTier::Tight;
    h.tier_name = "偏紧";
  } else {
    h.tier = HostTier::Critical;
    h.tier_name = "紧张";
  }
  return h;
}


}  // namespace tools
