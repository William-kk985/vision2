/**
 * @file utils/system/host_info.hpp
 * @brief ⭐⭐ **本机硬件画像 + 按核数给出的运行建议**（启动时打印一次）
 *
 * ## 为什么需要
 * 实测（i7-14700HX，28 逻辑核）：
 * ```
 *   OpenVINO 推理        5.2 ms（自动用 6~7 线程，加速比 3.28×）
 *   自瞄线程合计          ~7 ms      ← 只占 30fps 的 33 ms 预算 21%
 *   进程总 CPU            375%  =  3.75 核
 * ```
 * ⭐ 结论：**帧率由相机决定（30fps → 33 ms），处理只用 8~9 ms** ——
 *    所以**核少也不会掉帧率**，真正的风险是 **OpenVINO 抢核**：
 * ```
 *   4 核机器：自瞄 1.0 + OpenVINO 2.5 + 相机/SDK 0.5  ≈ 4.0 核  ⚠️ 饱和
 * ```
 * ⚠️ 而 **sink 的 worker 会再抢核**（尤其「录像」的 MJPG 编码 ≈ 1 核、
 *    「存图」的 PNG 编码 ≈ 0.1~0.2 核）→ 4 核上就可能开始抖动。
 *
 * ## 实测参考（同一台机器，改变 OpenVINO 线程数）
 * | 线程 | 推理 |
 * |---|---|
 * | 1 | 16.89 ms |
 * | 4 | 7.28 ms |
 * | 8 | 5.39 ms |
 * | **自动** | ⭐ **5.15 ms**（最快） |
 * ⇒ **别手动限线程**（设 12 反而 7.39 ms）；核少时用默认即可。
 */
#ifndef HZMIR_UTILS_SYSTEM_HOST_INFO_HPP
#define HZMIR_UTILS_SYSTEM_HOST_INFO_HPP

#include <string>

namespace tools
{

/// 核数档位（决定建议的严格程度）
enum class HostTier
{
  Roomy,    ///< ⭐ ≥16 核：宽松，随便开 sink
  Okay,     ///< ✅ 8~15 核：够用，窗口/录像慎开
  Tight,    ///< 🔶 4~7 核：够用但紧张，只留 CSV / PlotJuggler
  Critical  ///< ⚠️ ≤3 核：紧张，全关 sink
};

/// 本机画像
struct HostInfo
{
  unsigned logical_cores = 0;                         ///< 逻辑核数
  HostTier tier = HostTier::Okay;                     ///< 档位
  const char * tier_name = "?";                       ///< 档位中文名
  std::string advice;                                 ///< 建议（多行，已含缩进）
};

/// @brief 探测本机（`std::thread::hardware_concurrency()`）
HostInfo detect_host();

/// @brief ⭐⭐ 启动时打印「本机核数 + 建议配置」—— 四兵种各调一次
/// @param robot 兵种名（用于日志前缀）
void print_host_advice(const char * robot);

}  // namespace tools

#endif  // HZMIR_UTILS_SYSTEM_HOST_INFO_HPP
