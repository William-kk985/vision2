/**
 * @file utils/debug/hotkeys.hpp
 * @brief ⭐ 终端热键控制台（doc 09 §14.6 按键表）
 *
 * **原理**：把 stdin 切到 raw 非阻塞模式（`termios`，`VMIN=0/VTIME=0`），
 * 主循环每帧 `poll()` 一次即可 —— **不需要 GUI，也不需要 `cv::waitKey`**。
 *
 * ⚠️ 不是 tty（管道/重定向）时**自动禁用**，`poll()` 恒返回 `'\0'`，不影响跑批与测试。
 *
 * 按键表（与 doc 09 §14.6 对齐）：
 * ```
 *   1  可视化窗口 开/关      2  CSV 开/关        3  PlotJuggler 开/关
 *   4  存图（L3）             d  日志级别循环      p  暂停/继续
 *   r  热重载配置             q  退出
 * ```
 * ⚠️ `1` / `4` 依赖「图像通道」（L3），本阶段先留接口（见 doc 10 W19）。
 */
#ifndef HZMIR_UTILS_DEBUG_HOTKEYS_HPP
#define HZMIR_UTILS_DEBUG_HOTKEYS_HPP

#include <functional>
#include <string>
#include <unordered_map>
#include <utility>
#include <vector>

#include "utils/concurrency/exiter.hpp"
#include "utils/debug/debug_sink.hpp"
#include "utils/log/logger.hpp"

namespace tools
{

class HotkeyConsole
{
public:
  HotkeyConsole();
  ~HotkeyConsole();
  HotkeyConsole(const HotkeyConsole &) = delete;
  HotkeyConsole & operator=(const HotkeyConsole &) = delete;

  /// @brief stdin 是否为 tty（false = 已禁用）
  bool enabled() const { return enabled_; }

  /// @brief 非阻塞取一键；无键返回 `'\0'`
  /// @brief 主循环每帧调用。⭐ **内部会限频**（默认每 3 帧才真正 `read()` 一次）
  ///
  /// ## 为什么要限频（W46 实测）
  /// ```
  /// 非 tty（重定向跑批）: poll() =   1.3 ns   ← 走禁用快路径，连 read() 都不调
  /// 真 tty（实车终端）  : poll() = 195.3 ns   ← ⚠️ 一次 read() 系统调用
  /// ```
  /// ⭐ **195 ns 是常驻开销里第二大的项**（仅次于 `Expense`）。
  /// 而**人手按键根本不需要 100 Hz 响应** —— 每 3 帧问一次（~33 Hz）人感知不出，
  /// 开销直接降到 **~65 ns**。
  char poll();

  /// @brief 设置轮询间隔（帧）。`1` = 每帧都问（旧行为）
  void set_poll_interval(int frames) { poll_every_ = frames < 1 ? 1 : frames; }
  int poll_interval() const { return poll_every_; }

  /// @brief ⭐ 直接注入一个按键（**测试用** / 非 tty 来源，如串口、网络、GUI）
  /// @return 该键是否已注册并被派发
  bool feed(char key);

  /// @brief 注册按键回调
  void on(char key, const std::string & desc, std::function<void()> fn);

  /// @brief 帮助文本（启动时打印）
  std::string help() const;

  /// @brief 已派发的按键次数（测试用）
  size_t dispatched() const { return dispatched_; }

private:
  bool enabled_ = false;
  int poll_every_ = 3;   // ⭐ W46：每 3 帧才真 read() 一次（~33 Hz，人手够用）
  int poll_tick_ = 0;
  bool saved_ = false;
  std::unordered_map<char, std::function<void()>> handlers_;
  std::vector<std::pair<char, std::string>> descs_;
  size_t dispatched_ = 0;
};

/// @brief ⭐ 把「标准 Debug 按键」绑到 `SinkHub`
///
/// `1`/`4` 需要「图像通道」—— 本阶段留空（W19 补 L3 image sink）
/// @param paused 传入非空则绑定 `p` 暂停开关
struct DebugKeyBindings
{
  /// @brief sink 工厂：按键时**按需创建**（不按键就一个字节都不占）
  using SinkFactory = std::function<std::shared_ptr<IDebugSink>()>;

  /// ⭐ 四个通道的工厂。空工厂 = 该键不可用（会明确提示，不静默失败）
  ///   csv        按键 2   —— 常驻层，便宜
  ///   plotjuggler 按键 3   —— 常驻层，便宜
  ///   window     按键 1   —— ⭐ L3，贵（需要 DISPLAY）
  ///   image      按键 4   —— ⭐ L3，贵（落盘）
  struct SinkFactories
  {
    SinkFactory csv;
    SinkFactory plotjuggler;
    SinkFactory window;
    SinkFactory image;
  };

  /// @param on_reload 按键 `r` 的回调（配置热重载）；为空则提示不可用
  static void bind(
    HotkeyConsole & hk, SinkHub & hub, const SinkFactories & factories, bool * paused = nullptr,
    std::function<void()> on_reload = nullptr);
};

}  // namespace tools

#endif  // HZMIR_UTILS_DEBUG_HOTKEYS_HPP
