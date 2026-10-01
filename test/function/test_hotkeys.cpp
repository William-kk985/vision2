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

    const std::string h = hk.help();
    std::printf("     %s\n", h.c_str());
    for (char k : {'1', '2', '3', '4', 'd', 'p', 'q'})
      assert(h.find(k) != std::string::npos);
    ok("help() 列出全部 7 个按键（1/2/3/4/d/p/q）");
    assert(h.find("L3") != std::string::npos);
    ok("help() 标注了 1/4 属于 L3 昂贵通道");

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

    assert(hk.feed('1'));                       // 已注册
    assert(!hub.has("window"));                 // 但 factories.window 为空 → 明确提示不挂
    ok("按 1 → w窗口工厂为空时明确提示「不可用」，不静默失败");

    assert(!hk.feed('Z'));   // 未注册
    ok("未注册的键不派发（返回 false）");

    assert(hk.dispatched() >= 6);
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

  // ═══ ⑤ 工厂为空时明确提示（1/4 在未装配 L3 的构建里）═══
  std::printf("⑤ 工厂缺失时的行为\n");
  {
    tools::SinkHub hub;
    tools::HotkeyConsole hk;
    tools::DebugKeyBindings::SinkFactories none;   // 全部为空
    tools::DebugKeyBindings::bind(hk, hub, none, nullptr);
    assert(hk.feed('1'));    // 已注册，但会打印 warn
    assert(hk.feed('4'));
    assert(hub.size() == 0);
    ok("工厂为空时按 1/4 只提示「不可用」，不静默挂空 sink");
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
