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
  char poll();

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
