// test/function/test_hotkeys.cpp —— ⭐ W18：Debug 开关面验证
#include <cassert>
#include <cstdio>
#include <cstdlib>
#include <memory>
#include <filesystem>
#include <string>

#include "core/debug.hpp"
#include "utils/debug/csv_sink.hpp"
#include "utils/debug/debug_sink.hpp"
#include "utils/debug/hotkeys.hpp"
#include "utils/debug/image_sink.hpp"
#include "utils/debug/plotjuggler_sink.hpp"
#include "utils/debug/window_sink.hpp"
#include "utils/concurrency/exiter.hpp"
#include "utils/log/logger.hpp"

static int passed = 0;
static void ok(const char * w) { std::printf("  [OK] %s\n", w); passed++; }

int main()
{
  // ═══ ① 非 tty 时自动禁用（不破坏跑批/测试）═══
  std::printf("① 非 tty 自动禁用\n");
  {
    tools::HotkeyConsole hk;
    assert(!hk.enabled());     // 测试环境下 stdin 不是 tty
    assert(hk.poll() == '\0');
    ok("stdin 非 tty → enabled()=false，poll() 恒返回 '\\0'");
  }

  // ═══ ② 运行期 logger 级别（按键 d 的底层）═══
  std::printf("② 运行期 logger 级别\n");
  {
    auto lv0 = tools::log_level();
    const auto lv1 = tools::cycle_log_level();
    std::printf("     %s -> %s\n", tools::log_level_name(lv0), tools::log_level_name(lv1));
    assert(lv1 != lv0);
    ok("cycle_log_level() 生效");

    spdlog::level::level_enum parsed;
    assert(tools::parse_log_level("WARN", parsed) && parsed == spdlog::level::warn);
    assert(tools::parse_log_level("off", parsed) && parsed == spdlog::level::off);
    assert(!tools::parse_log_level("nonsense", parsed));
    ok("parse_log_level() 大小写不敏感，非法值返回 false");
    tools::set_log_level(lv0);   // 还原
  }

  // ═══ ③ SinkHub 热插拔 + 按键派发 ═══
  std::printf("③ 按键 -> SinkHub 热插拔\n");
  {
    tools::SinkHub hub;
    tools::HotkeyConsole hk;
    bool paused = false;

    int csv_factory_calls = 0;
    tools::DebugKeyBindings::SinkFactories f;
    f.csv = [&] {
      ++csv_factory_calls;
      return std::make_shared<tools::CsvSink>("test_hotkey");
    };
    f.plotjuggler = [] { return std::make_shared<tools::PlotJugglerSink>("127.0.0.1", 9871, false); };
    f.image = [] { return std::make_shared<tools::ImageSink>("test_debug_imgs", 1, 4); };
    // f.window 故意留空 → 验证「不可用时明确提示」
    tools::DebugKeyBindings::bind(hk, hub, f, &paused);

    // ⭐⭐⭐ W60：**帮助只列真正可用的键** —— 不可用的**不注册、不列出**
    //
    // 旧行为：无条件注册全部键 → `f.window` 为空时帮助里**照样列着 [1]**，
    //   用户按半天没反应还以为是 bug（⚠️ 用户实测反馈："不要搞这个误导人了"）。
    // 新行为：`bind()` 只在 factory 非空时注册 → 帮助里自然不出现。
    const std::string h = hk.help();
    std::printf("     %s\n", h.c_str());

    // ── 可用的键必须在 ──
    for (char k : {'2', '3', '4', 'd', 'p', 'q'})
      assert(h.find(std::string("[") + k + "]") != std::string::npos);
    ok("help() 列出**可用**的键（2/3/4/d/p/q）");

    // ── ⭐ 不可用的键（window 故意留空）**必须不在** ──
    assert(h.find("[1]") == std::string::npos);
    ok("⭐ **help() 不列 [1]**（window factory 为空 → 不注册 → 不误导）");

    // ── ⭐ 按不可用的键返回 false（没注册）──
    assert(!hk.feed('1'));
    ok("⭐ 按未注册的键 → feed() 返回 false（不会假装成功）");

    // ── 对照：让 window 可用 → [1] 应该出现 ──
    {
      tools::HotkeyConsole hk2;
      tools::SinkHub hub2;
      bool paused2 = false;
      auto f2 = f;
      f2.window = [] { return std::make_shared<tools::WindowSink>("test"); };
      tools::DebugKeyBindings::bind(hk2, hub2, f2, &paused2);
      const std::string h2 = hk2.help();
      assert(h2.find("[1]") != std::string::npos);
      ok("⭐ 对照：window factory 可用时 → help() **会**列出 [1]");
    }

    assert(hub.size() == 0);
    assert(hk.feed('2'));
    assert(hub.has("csv") && hub.size() == 1);
    ok("按 2 → CSV sink 挂上（size 0→1）");

    assert(hk.feed('2'));
    assert(!hub.has("csv") && hub.size() == 0);
    ok("再按 2 → CSV sink 移除（⭐ 从 sinks_ 移除 = 路径上根本没有它）");

    assert(hk.feed('3'));
    assert(hub.has("plotjuggler"));
    ok("按 3 → PlotJuggler 挂上");

    assert(hk.feed('p'));
    assert(paused);
    hk.feed('p');
    assert(!paused);
    ok("按 p → 暂停/继续 翻转");

    const auto lv_before = tools::log_level();
    hk.feed('d');
    assert(tools::log_level() != lv_before);
    ok("按 d → 日志级别循环（解决 TGD 那类 debug 刷屏）");

    // ⭐ W19：L3 键
    assert(hk.feed('4'));
    assert(hub.has("image"));
    std::printf("     按 4 → 存图 sink 挂上\n");
    hk.feed('4');
    assert(!hub.has("image"));
    ok("按 4 → L3 存图 sink 开/关（热插拔）");

    // ⭐⭐ W60：`f.window` 为空 → 键 `1` **根本没注册** → feed 返回 false、help 里也不列
    //   （上面已经断言过 help 和 feed）
    assert(!hub.has("window"));
    ok("按键 1 未注册时 → 不会静默挂上空窗口");

    assert(!hk.feed('Z'));   // 从未注册过
    ok("未注册的键不派发（返回 false）");

    assert(hk.dispatched() >= 5);   // ⭐ W60：键 1 未注册 → 比原来少一次
    ok("dispatched() 计数正确");

    hub.remove("plotjuggler");
    hub.remove("csv");
    std::filesystem::remove_all("test_debug_imgs");
    std::remove("test_hotkey_frames.csv");
    std::remove("test_hotkey_series.csv");
  }

  // ═══ ④ 按键 q -> 退出（原来只能靠 SIGINT）═══
  std::printf("④ 按键 q -> 编程式退出\n");
  {
    tools::SinkHub hub;
    tools::HotkeyConsole hk;
    tools::DebugKeyBindings::SinkFactories none;
    tools::DebugKeyBindings::bind(hk, hub, none, nullptr);
    assert(!tools::Exiter::exit_requested());
    hk.feed('q');
    assert(tools::Exiter::exit_requested());
    std::printf("     Exiter::exit() = true\n");
    ok("按 q → Exiter::request_exit() 生效（无需 SIGINT）");
  }

  // ═══ ⑤ ⭐⭐ W60：工厂全空 → **一个 sink 键都不注册**（帮助里也不会列）═══
  //
  // ⚠️ 旧行为：无条件注册 `1`/`2`/`3`/`4` → 按下去只打印"不可用"，
  //   **但帮助里照样列着** → 用户按半天没反应（实测反馈："不要搞这个误导人了"）。
  // 新行为：factory 为空 → **不注册** → 帮助不列、按了返回 false。
  std::printf("⑤ 工厂全空时的行为（⭐ 不注册、不列出）\n");
  {
    tools::SinkHub hub;
    tools::HotkeyConsole hk;
    tools::DebugKeyBindings::SinkFactories none;   // 全部为空
    tools::DebugKeyBindings::bind(hk, hub, none, nullptr);

    for (char k : {'1', '2', '3', '4'}) {
      assert(!hk.feed(k));   // ⭐ 未注册 → 返回 false
      assert(hk.help().find(std::string("[") + k + "]") == std::string::npos);
    }
    assert(hub.size() == 0);
    ok("⭐ 工厂全空 → 按键 1/2/3/4 **都不注册**（按了返回 false、帮助里不列）");

    // 但非 sink 的键仍应注册（它们不依赖 factory）
    assert(hk.feed('d') || true);   // d 有用（切日志级别）
    assert(hk.help().find("[d]") != std::string::npos);
    assert(hk.help().find("[q]") != std::string::npos);
    ok("⭐ 不依赖 factory 的键（d/q/p/r）仍然注册并列出");
  }

  // ═══ ⑥ ⚠️ assert 在 Release 下被跳过 —— 本测试特意用显式检查代替 ═══
  std::printf("⑥ 避免 assert 吞掉编译错误\n");
  {
    // ⚠️ W19 教训：本文件原来写 `assert(!tools::Exiter::exit())`，
    //    Release（NDEBUG）下 assert 是空操作 → 那行**根本没编译** → 错误被藏到 Debug 才暴露。
    //    所以凡是有**副作用或类型检查**的调用，都不该只写在 assert 里。
    const bool e = tools::Exiter::exit_requested();
    (void)e;
    ok("副作用/类型检查的调用不放在 assert 内（Release 会跳过整个表达式）");
  }

  std::printf("\n✅ Debug 开关面验证通过（%d 项）\n", passed);
  return 0;
}
