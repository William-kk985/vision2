#include "utils/system/host_info.hpp"

#include <cstdlib>
#include <thread>

#include "utils/log/logger.hpp"

namespace tools
{

HostInfo detect_host()
{
  HostInfo h;
  h.logical_cores = std::thread::hardware_concurrency();
  if (h.logical_cores == 0) h.logical_cores = 1;   // 探测失败 → 按最保守算

  // ⭐ 覆盖：`HZMIR_CORES=<n>` —— 用于 ① 测试各档位 ② 用户手动指定
  //   （`hardware_concurrency()` 返回的是【整机逻辑核】，不受 taskset/cgroup 影响，
  //    所以想在小核机器上验证建议，或者容器里被限核时，都需要这个口子）
  if (const char * env = std::getenv("HZMIR_CORES"); env && *env) {
    const int v = std::atoi(env);
    if (v > 0) {
      h.logical_cores = static_cast<unsigned>(v);
      tools::logger()->info("[host] ⚠️ 核数被 HZMIR_CORES 覆盖为 {}", v);
    }
  }

  const unsigned n = h.logical_cores;
  if (n >= 16) {
    h.tier = HostTier::Roomy;
    h.tier_name = "宽松";
    h.advice =
      "      · 推理会自动用多核（实测加速比上限 ~3.3×，8 线程后饱和）\n"
      "      · sink 随便开：CSV / PlotJuggler / 存图 / 录像都不影响帧率\n"
      "      ⚠️ 唯一别开的是【窗口】（cv::imshow 在自瞄线程是 ms 级 + 阻塞）";
  } else if (n >= 8) {
    h.tier = HostTier::Okay;
    h.tier_name = "够用";
    h.advice =
      "      · 帧率由相机决定（30fps → 33 ms），处理只用 ~8 ms → 余量充足\n"
      "      · 推荐：CSV / PlotJuggler 可开；⚠️ 窗口 / 录像慎开（录像 MJPG 编码吃 ~1 核）";
  } else if (n >= 4) {
    h.tier = HostTier::Tight;
    h.tier_name = "够用但紧张";
    h.advice =
      "      ⚠️ OpenVINO 推理在此核数会占 ~2.5 核 → 加上自瞄/相机已接近饱和\n"
      "      ⭐ 推荐：只开 CSV / PlotJuggler（各 ~0.01 核）\n"
      "      ❌ 别开：窗口（ms 级阻塞）/ 录像（~1 核）/ 存图（~0.15 核）\n"
      "      · 不要手动限 OpenVINO 线程（实测设 12 反而慢 43%）";
  } else {
    h.tier = HostTier::Critical;
    h.tier_name = "紧张";
    h.advice =
      "      ⚠️⚠️ 核数很少：推理会显著变慢（4 线程实测 7.3 ms，1 线程 16.9 ms）\n"
      "      ⭐ 建议：关掉全部 sink（不传 --csv/--pj/--record，不按 1/4）\n"
      "      · 若仍超预算，考虑把 yaml 的 `fps` 降到 20（放宽到 50 ms）";
  }
  return h;
}

void print_host_advice(const char * robot)
{
  const auto h = detect_host();
  tools::logger()->info(
    "[{}] ⭐ 本机 {} 逻辑核 → 档位【{}】\n{}", robot, h.logical_cores, h.tier_name, h.advice);
}

}  // namespace tools
