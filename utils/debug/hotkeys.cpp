#include "utils/debug/hotkeys.hpp"

#include <termios.h>
#include <unistd.h>

#include <cstdio>
#include <functional>
#include <sstream>

#include "utils/log/logger.hpp"

namespace tools
{
namespace
{
// 原始终端设置（进程内单例 —— 全局只应有一个 HotkeyConsole）
termios g_orig_termios{};
bool g_has_orig = false;
}  // namespace

HotkeyConsole::HotkeyConsole()
{
  if (!::isatty(STDIN_FILENO)) return;   // 管道/重定向 → 禁用

  termios raw{};
  if (::tcgetattr(STDIN_FILENO, &raw) != 0) return;

  g_orig_termios = raw;
  g_has_orig = true;

  raw.c_lflag &= static_cast<tcflag_t>(~(ICANON | ECHO));
  raw.c_cc[VMIN] = 0;    // 不阻塞
  raw.c_cc[VTIME] = 0;
  if (::tcsetattr(STDIN_FILENO, TCSANOW, &raw) != 0) return;

  enabled_ = true;
  saved_ = true;
}

HotkeyConsole::~HotkeyConsole()
{
  if (enabled_ && g_has_orig) {
    ::tcsetattr(STDIN_FILENO, TCSANOW, &g_orig_termios);
    g_has_orig = false;
  }
}

char HotkeyConsole::poll()
{
  if (!enabled_) return '\0';
  // ⭐⭐ W46：**限频** —— 真终端下一次 `read()` 要 ~195 ns（实测），
  //   而人手按键不需要 100 Hz 响应。每 `poll_every_` 帧才真问一次。
  if (++poll_tick_ < poll_every_) return '\0';
  poll_tick_ = 0;
  unsigned char c = 0;
  const ssize_t n = ::read(STDIN_FILENO, &c, 1);
  if (n != 1) return '\0';
  return feed(static_cast<char>(c)) ? static_cast<char>(c) : '\0';
}

bool HotkeyConsole::feed(char key)
{
  auto it = handlers_.find(key);
  if (it == handlers_.end()) return false;   // 未注册的键不派发
  ++dispatched_;
  it->second();
  return true;
}

void HotkeyConsole::on(char key, const std::string & desc, std::function<void()> fn)
{
  handlers_[key] = std::move(fn);
  descs_.emplace_back(key, desc);
}

std::string HotkeyConsole::help() const
{
  std::ostringstream o;
  o << "热键: ";
  for (size_t i = 0; i < descs_.size(); ++i) {
    if (i) o << "  ";
    o << "[" << descs_[i].first << "] " << descs_[i].second;
  }
  if (!enabled_) o << "   （stdin 非 tty → 热键已禁用）";
  return o.str();
}

// ═══════════════════════════════════════════════════════════════
// 标准 Debug 按键绑定（doc 09 §14.6）
// ═══════════════════════════════════════════════════════════════
namespace
{
/// 统一的「开/关一个 sink」逻辑（⭐ 热插拔：关掉 = 从 sinks_ 移除 = 路径上根本没有它）
void toggle_sink(
  SinkHub & hub, const char * sink_name, const DebugKeyBindings::SinkFactory & factory,
  const char * label)
{
  if (hub.has(sink_name)) {
    hub.remove(sink_name);
    tools::logger()->warn("[hotkey] {} 已移除（后续 on_frame/on_image 路径上没有它）", label);
    return;
  }
  if (!factory) {
    // ⭐⭐ W55：说清楚**为什么**不可用 + **怎么办**（原来只一句"不可用"）
    tools::logger()->warn(
      "[hotkey] ⚠️ {} **当前环境不可用**。\n"
      "    常见原因：无显示环境（`DISPLAY` 未设置）→ 可视化窗口开不了（`cv::imshow` 需要 X）。\n"
      "    可选：① 设好 `DISPLAY` 再跑（本地图形界面）\n"
      "          ② 用 `--debug-img` 存图代替（不需要显示）\n"
      "          ③ 远程/无头机器：用 CSV / PlotJuggler 看数据（L0~L2 不依赖显示）",
      label);
    return;
  }
  hub.add(factory());
  tools::logger()->warn("[hotkey] {} 已挂上", label);
}
}  // namespace

void DebugKeyBindings::bind(
  HotkeyConsole & hk, SinkHub & hub, const SinkFactories & f, bool * paused,
  std::function<void()> on_reload)
{
  // ⭐⭐⭐ W60：**只注册真正可用的键** —— 不可用的**不进帮助文本**
  //
  // 原来无条件 `hk.on('1'/'4', ...)`，然后 `toggle_sink` 在 factory 为空时打印
  // "不可用"。⚠️ **但帮助里照样列着它们** → 用户按了半天没反应，还以为是 bug。
  // 现在：factory 为空 → **不注册** → 帮助里自然不出现。**不再误导。**
  auto reg = [&hk](char key, const char * desc, DebugKeyBindings::SinkFactory fac,
                   std::function<void()> act) {
    if (!fac) return;   // ⭐ 不可用 → 不注册（帮助里就不会列出）
    hk.on(key, desc, std::move(act));
  };

  reg('2', "CSV 开/关", f.csv, [&hub, f] { toggle_sink(hub, "csv", f.csv, "CSV sink"); });
  reg('3', "PlotJuggler 开/关", f.plotjuggler,
      [&hub, f] { toggle_sink(hub, "plotjuggler", f.plotjuggler, "PlotJuggler sink"); });
  reg('1', "可视化窗口 开/关", f.window,
      [&hub, f] { toggle_sink(hub, "window", f.window, "窗口"); });
  reg('4', "存图 开/关", f.image, [&hub, f] { toggle_sink(hub, "image", f.image, "存图"); });

  // d —— 日志级别循环
  hk.on('d', "日志级别循环", [] {
    tools::logger()->warn("[hotkey] 日志级别 -> {}", tools::log_level_name(tools::cycle_log_level()));
  });

  // p —— 暂停/继续
  if (paused) {
    hk.on('p', "暂停/继续", [paused] {
      *paused = !*paused;
      tools::logger()->warn("[hotkey] {}", *paused ? "已暂停" : "已继续");
    });
  }

  // r —— ⭐ W21：配置热重载
  hk.on('r', "热重载配置", [on_reload] {
    if (!on_reload) {
      tools::logger()->warn("[hotkey] 热重载未装配");
      return;
    }
    on_reload();
  });

  // q —— 退出
  hk.on('q', "退出", [] {
    tools::logger()->warn("[hotkey] 请求退出");
    Exiter::request_exit();
  });

}

}  // namespace tools
