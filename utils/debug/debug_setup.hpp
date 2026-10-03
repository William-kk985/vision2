/**
 * @file utils/debug/debug_setup.hpp
 * @brief ⭐⭐ W49（改进方案 D）：**一行装配整个 Debug 体系**
 *
 * ## 解决什么问题
 * 原来 4 个兵种 main 各**手写 ~30 行**装配代码：
 * ```cpp
 * tools::SinkHub hub;
 * tools::Expense expense;
 * if (!csv_prefix.empty()) hub.add(std::make_shared<CsvSink>(csv_prefix));
 * hub.add(std::make_shared<PlotJugglerSink>("127.0.0.1", 9870, cli.get<bool>("pj")));
 * tools::DebugKeyBindings::SinkFactories factories;
 * factories.csv = ...; factories.plotjuggler = ...;
 * factories.image = ...; if (WindowSink::available()) factories.window = ...;
 * if (cli.get<bool>("debug-img") && factories.image) hub.add(factories.image());
 * ...
 * tools::DebugKeyBindings::bind(hotkeys, hub, factories, &paused, reload_cb);
 * tools::logger()->info("{}", hotkeys.help());
 * ```
 * ⭐ **4 遍重复 → 加一个新 sink 要改 4 处、容易漏配。**
 *
 * ## 现在
 * ```cpp
 * tools::DebugRuntime dbg({.csv_prefix = csv_prefix, .pj = cli.get<bool>("pj"),
 *                          .img = ..., .window = ..., .name = "infantry"},
 *                         [&] { hot_reloader.reload_if_changed(); });
 * // 循环里：dbg.hotkeys.poll();  if (dbg.paused) continue;
 * //         dbg.expense.begin("perceive");  ...  dbg.hub.on_frame(fd);
 * ```
 *
 * ## ⚠️ 宏规范
 * `#ifdef DEBUG_L3_ENABLE` **只出现在本文件（装配函数）** —— 符合「`#ifdef` 只在
 * `config.hpp` / 装配函数 / `CMakeLists.txt` / 测试入口」的纪律。
 * ⚠️ **热路径的 L3 门控**（`hub.wants_image()` 后构造 overlay）**仍留在各 main** ——
 * 那是**业务相关**的（各兵种的 overlay 画法不同）。
 * ⭐ **W61 起主程序里也不再有 `#ifdef`**（原来那段被宏包着 → Release 下窗口永远黑屏）。
 */
#ifndef HZMIR_UTILS_DEBUG_DEBUG_SETUP_HPP
#define HZMIR_UTILS_DEBUG_DEBUG_SETUP_HPP

#include <functional>
#include <memory>
#include <string>
#include <utility>

#include "utils/system/paths.hpp"   // W87: unified output/ layout
#include "utils/debug/csv_sink.hpp"
#include "utils/debug/debug_sink.hpp"
#include "utils/debug/expense.hpp"
#include "utils/debug/hotkeys.hpp"
#include "utils/debug/plotjuggler_sink.hpp"
#include "utils/log/logger.hpp"

// ⭐⭐⭐ W60：**L3 无条件编入** —— 原来它在 `#ifdef DEBUG_L3_ENABLE` 里，导致
//   Release 构建下按键 `1`/`4` **永远按不动、还得重编**。
//   ⚠️ 但实测 `libopencv_highgui` **本来就已经链接**（`ldd` 可查），
//   所以那个宏**没有任何"省依赖/省体积"的收益**，只有害处。
//   ⇒ 现在 L3 总是编入，**纯运行期**控制（按键/`--debug-img`/`--debug-window`）。
#include "utils/debug/image_sink.hpp"
#include "utils/debug/window_sink.hpp"

namespace tools
{

/// @brief 装配选项（指定初始化器风格：`{.csv_prefix = p, .pj = true}`）
struct DebugOptions
{
  std::string csv_prefix;              ///< 空 = 不落 CSV
  bool pj = false;                     ///< 启动就发 PlotJuggler UDP
    // ⭐ W88：**跨机器看** —— 目标地址可配（默认本机）。
    //   对方机器上 PlotJuggler 的 UDP Server 监听 `0.0.0.0`（默认就是），
    //   这里填它的 IP 即可；⚠️ 记得放行对方防火墙的 UDP 9870。
    //   ⭐ 流量很小：一帧 JSON ≈ 400 B × 30 fps ≈ **12 KB/s**，WiFi 完全够。
    std::string pj_host = "127.0.0.1";   ///< 目标 IP（跨机器时填对方 IP）
    uint16_t pj_port = 9870;             ///< 目标端口
  bool img = false;                    ///< L3：启动就开存图
  bool window = false;                 ///< L3：启动就开可视化窗口
  bool verbose_hotkeys = true;         ///< 打印热键帮助
  std::string name = "hzmir";          ///< 日志前缀 + 窗口标题
  std::string csv_fallback = "hzmir_hotkey";   ///< 热键开 CSV 时的默认前缀
  std::string img_dir = "output/images";   // ⭐ W87（默认值在这里；也可传别的）  ///< 存图目录
  int img_every_n = 30;                ///< 每 N 帧存 1 张
  size_t img_max_files = 500;          ///< 总共最多存几张
  size_t img_max_queue = 16;           ///< ⭐ 队列深度上限（防 OOM）
  bool img_deep_copy = true;           ///< ⭐ 深拷贝（安全）；上游每帧新建 Mat 时可设 false
};

/// @brief ⭐ 整个 Debug 运行期对象的聚合 —— 一行装配
struct DebugRuntime
{
  SinkHub hub;
  Expense expense;
  HotkeyConsole hotkeys;
  DebugKeyBindings::SinkFactories factories;
  bool paused = false;   ///< ⭐ 主循环读它决定是否 continue

  /// @param opt 装配选项
  /// @param on_reload 按 `r` 时的回调（配置热重载）；可为空
  explicit DebugRuntime(const DebugOptions & opt, std::function<void()> on_reload = nullptr)
  {
    // ── ① 启动就挂的 sink ──
    if (!opt.csv_prefix.empty()) hub.add(std::make_shared<CsvSink>(opt.csv_prefix));
    hub.add(std::make_shared<PlotJugglerSink>(opt.pj_host, opt.pj_port, opt.pj));

    // ── ② 热键用的工厂（按键时现场造 sink）──
    const std::string csv_prefix = opt.csv_prefix.empty() ? opt.csv_fallback : opt.csv_prefix;
    factories.csv = [csv_prefix] { return std::make_shared<CsvSink>(csv_prefix); };
    {
        const std::string h = opt.pj_host; const uint16_t pt = opt.pj_port;
        factories.plotjuggler = [h, pt] { return std::make_shared<PlotJugglerSink>(h, pt, true); };
      }

    // ── ③ L3 装配（⭐⭐ W60：**不再用 `#ifdef`** —— 总是编入，纯运行期控制）──
    const auto o = opt;   // 捕获副本
    factories.image = [o] {
      return std::make_shared<ImageSink>(
        o.img_dir, o.img_every_n, o.img_max_files, o.img_deep_copy, o.img_max_queue);
    };
    // ⭐ 窗口是**唯一**需要运行期检查的（无 DISPLAY 时 `cv::imshow` 会崩）
    if (WindowSink::available())
      factories.window = [o] { return std::make_shared<WindowSink>(o.name); };
    else
      tools::logger()->info(
        "[{}] 无显示环境（DISPLAY 未设置）→ 按键 1 不可用；按键 4 存图仍可用", opt.name);

    if (opt.img && factories.image) hub.add(factories.image());
    if (opt.window && factories.window) hub.add(factories.window());
    tools::logger()->debug(
      "[{}] L3 图像通道可用（存图={} 窗口={}）", opt.name,
      hub.has("image") ? "开" : "关", hub.has("window") ? "开" : "关");

    // ── ④ 热键绑定（`bind` 的 paused/on_reload 都有默认值）──
    DebugKeyBindings::bind(hotkeys, hub, factories, &paused, std::move(on_reload));

    if (opt.verbose_hotkeys) tools::logger()->info("{}", hotkeys.help());
  }

  DebugRuntime(const DebugRuntime &) = delete;
  DebugRuntime & operator=(const DebugRuntime &) = delete;
};

}  // namespace tools

#endif  // HZMIR_UTILS_DEBUG_DEBUG_SETUP_HPP
