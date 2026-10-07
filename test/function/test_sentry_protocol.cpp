/// @file test_sentry_protocol.cpp
/// @brief 哨兵串口协议的结构体布局验证。
///
/// 这是**整份协议的契约**：布局必须与下位机侧的 Python `struct` 逐字节一致，
/// 否则收发会静默错位（读到的 q/yaw 全是垃圾但不会报错）。
///
/// 覆盖：
///   ① 总长度（46 / 42）
///   ② **每个字段的偏移**（`offsetof` —— 比只查总长度强得多：
///      总长度对而顺序错是可能的，例如两个相邻 float 互换）
///   ③ 帧头/帧尾常量
///   ④ 编码规则：mode 的三态、帧尾写入
#include "io/board/sentry_serial/sentry_serial.hpp"
#include "io/board/serial_scan.hpp"

#include <cstddef>
#include <cstdio>
#include <cstring>

static int failures = 0;
#define CHECK(cond, msg)                                           \
  do {                                                             \
    if (!(cond)) { std::printf("  FAIL  %s\n", msg); ++failures; } \
    else         { std::printf("  ok    %s\n", msg); }             \
  } while (0)

using io::SentryToVisionPacket;
using io::VisionToSentryPacket;

int main()
{
  std::printf("  ==== test_sentry_protocol ====\n\n");

  // ── ① 总长度 ──
  std::printf("  -- 总长度 --\n");
  CHECK(sizeof(SentryToVisionPacket) == 46, "上行包 = 46 字节");
  CHECK(sizeof(VisionToSentryPacket) == 42, "下行包 = 42 字节");
  CHECK(io::SENTRY_RX_SIZE == 46 && io::SENTRY_TX_SIZE == 42, "公开常量与 sizeof 一致");

  // ── ② 上行包逐字段偏移（对照 Python `<2sB9fHHB2s`）──
  std::printf("\n  -- 上行包偏移（下位机 -> 上位机）--\n");
  CHECK(offsetof(SentryToVisionPacket, head) == 0x00, "head   @0x00");
  CHECK(offsetof(SentryToVisionPacket, mode) == 0x02, "mode   @0x02");
  CHECK(offsetof(SentryToVisionPacket, q) == 0x03, "q[4]   @0x03");
  CHECK(offsetof(SentryToVisionPacket, yaw) == 0x13, "yaw    @0x13");
  CHECK(offsetof(SentryToVisionPacket, yaw_vel) == 0x17, "yaw_vel@0x17");
  CHECK(offsetof(SentryToVisionPacket, pitch) == 0x1B, "pitch  @0x1B");
  CHECK(offsetof(SentryToVisionPacket, pitch_vel) == 0x1F, "pitch_v@0x1F");
  CHECK(offsetof(SentryToVisionPacket, bullet_speed) == 0x23, "bspd   @0x23");
  CHECK(offsetof(SentryToVisionPacket, bullet_count) == 0x27, "bcount @0x27");
  CHECK(offsetof(SentryToVisionPacket, health) == 0x29, "health @0x29");
  CHECK(offsetof(SentryToVisionPacket, game_state) == 0x2B, "gstate @0x2B");
  CHECK(offsetof(SentryToVisionPacket, tail) == 0x2C, "tail   @0x2C");

  // ── ③ 下行包逐字段偏移（对照 Python `<2sB9fB2s`）──
  std::printf("\n  -- 下行包偏移（上位机 -> 下位机）--\n");
  CHECK(offsetof(VisionToSentryPacket, head) == 0x00, "head   @0x00");
  CHECK(offsetof(VisionToSentryPacket, mode) == 0x02, "mode   @0x02");
  CHECK(offsetof(VisionToSentryPacket, yaw) == 0x03, "yaw    @0x03");
  CHECK(offsetof(VisionToSentryPacket, yaw_vel) == 0x07, "yaw_vel@0x07");
  CHECK(offsetof(VisionToSentryPacket, yaw_acc) == 0x0B, "yaw_acc@0x0B");
  CHECK(offsetof(VisionToSentryPacket, pitch) == 0x0F, "pitch  @0x0F");
  CHECK(offsetof(VisionToSentryPacket, pitch_vel) == 0x13, "pitch_v@0x13");
  CHECK(offsetof(VisionToSentryPacket, pitch_acc) == 0x17, "pitch_a@0x17");
  CHECK(offsetof(VisionToSentryPacket, nav_x) == 0x1B, "nav_x  @0x1B");
  CHECK(offsetof(VisionToSentryPacket, nav_y) == 0x1F, "nav_y  @0x1F");
  CHECK(offsetof(VisionToSentryPacket, nav_w) == 0x23, "nav_w  @0x23");
  CHECK(offsetof(VisionToSentryPacket, status_position) == 0x27, "status @0x27");
  CHECK(offsetof(VisionToSentryPacket, tail) == 0x28, "tail   @0x28");

  // ── ④ 帧头/帧尾常量 ──
  std::printf("\n  -- 帧常量 --\n");
  CHECK(io::FRAME_HEAD[0] == 0x53 && io::FRAME_HEAD[1] == 0x50, "帧头 = 'S','P' (0x53 0x50)");
  CHECK(io::FRAME_TAIL[0] == 0xBB && io::FRAME_TAIL[1] == 0x66, "帧尾 = 0xBB 0x66");

  // ── ⑤ 打包后的实际字节（不依赖 struct 内存，手工按协议拼一遍再比对）──
  std::printf("\n  -- 打包字节序（小端 float）--\n");
  {
    VisionToSentryPacket p{};
    p.head[0] = io::FRAME_HEAD[0];
    p.head[1] = io::FRAME_HEAD[1];
    p.mode = 2;             // control + fire
    p.yaw = 1.0f;           // 小端 IEEE754: 00 00 80 3F
    p.tail[0] = 0xBB; p.tail[1] = 0x66;

    const auto * raw = reinterpret_cast<const uint8_t *>(&p);
    CHECK(raw[0] == 0x53 && raw[1] == 0x50, "raw[0..1] = 'S','P'");
    CHECK(raw[2] == 2, "raw[2] = mode");
    CHECK(raw[3] == 0x00 && raw[4] == 0x00 && raw[5] == 0x80 && raw[6] == 0x3F,
          "raw[3..6] = 1.0f 小端 (00 00 80 3F)");
    CHECK(raw[40] == 0xBB && raw[41] == 0x66, "raw[40..41] = 帧尾");
  }

  // ── ⑥ ⭐ 无导航时这一帧【照发】：nav 全 0，但控制字段完整 ──
  std::printf("\n  -- 无导航场景（导航包未就绪）--\n");
  {
    // 模拟：没有导航数据，只发云台控制
    VisionToSentryPacket p{};
    p.head[0] = io::FRAME_HEAD[0];
    p.head[1] = io::FRAME_HEAD[1];
    p.mode = 1;                 // 控制云台、不开火
    p.yaw = 0.5f;
    p.yaw_vel = 1.0f;
    p.yaw_acc = 2.0f;
    p.pitch = -0.2f;
    p.pitch_vel = 0.5f;
    p.pitch_acc = 1.0f;
    // ⭐ nav_* 与 status_position 保持默认 0（= 无导航）
    p.tail[0] = io::FRAME_TAIL[0];
    p.tail[1] = io::FRAME_TAIL[1];

    CHECK(p.nav_x == 0.0f && p.nav_y == 0.0f && p.nav_w == 0.0f, "无导航 ⇒ nav_x/y/w 全 0");
    CHECK(p.status_position == 0, "未启用 status_position ⇒ 0");
    CHECK(p.mode == 1 && p.yaw == 0.5f && p.pitch == -0.2f, "⭐ 云台控制字段【照发】（不依赖导航）");
    CHECK(p.yaw_vel == 1.0f && p.yaw_acc == 2.0f && p.pitch_vel == 0.5f && p.pitch_acc == 1.0f,
          "⭐ 含前馈的 vel/acc 也照发");
    CHECK(sizeof(p) == io::SENTRY_TX_SIZE, "无导航时帧长仍为 42（不缩短）");
  }

  // ── ⑦ ⭐ 有导航时填入真实数据 ──
  std::printf("\n  -- 有导航场景（导航包就绪后）--\n");
  {
    VisionToSentryPacket p{};
    p.mode = 2;
    p.yaw = 0.1f;
    p.nav_x = 1.5f;
    p.nav_y = -2.0f;
    p.nav_w = 0.785f;   // 45°
    p.status_position = 0;
    CHECK(p.nav_x == 1.5f && p.nav_y == -2.0f && p.nav_w == 0.785f, "⭐ 导航就绪 ⇒ 填入真实 nav");
    CHECK(p.mode == 2 && p.yaw == 0.1f, "导航就绪不影响控制字段");
  }

  // ── ⑧ `SentryMode` 枚举值（对齐下位机的 0/1/2/3）──
  std::printf("\n  -- 模式枚举 --\n");
  CHECK(static_cast<int>(io::SentryMode::IDLE) == 0, "IDLE = 0");
  CHECK(static_cast<int>(io::SentryMode::AUTO_AIM) == 1, "AUTO_AIM = 1");
  CHECK(static_cast<int>(io::SentryMode::SMALL_BUFF) == 2, "SMALL_BUFF = 2");
  CHECK(static_cast<int>(io::SentryMode::BIG_BUFF) == 3, "BIG_BUFF = 3");

  // ── ⑨ ⭐ 导航状态码常量（"没有导航"是 4，不是 0）──
  std::printf("\n  -- 导航状态码 --\n");
  CHECK(io::SENTRY_NAV_NO_NAV == 4, "SENTRY_NAV_NO_NAV = 4（无导航/待机）");
  CHECK(io::SENTRY_NAV_NO_NAV != 0, "无导航不是 0（0 是残血回血中）");

  // ── ⑩ ⭐ 串口自适应匹配逻辑 ──
  std::printf("\n  -- 串口自适应匹配 --\n");
  {
    using io::SerialPortInfo;
    // 造 3 个假设备：两个 CH340、一个 CP210x（带序列号）
    SerialPortInfo a;
    a.dev = "/dev/ttyUSB0"; a.vid = "1a86"; a.pid = "7523"; a.driver = "ch341";
    SerialPortInfo b;
    b.dev = "/dev/ttyUSB1"; b.vid = "1a86"; b.pid = "7523"; b.driver = "ch341";
    SerialPortInfo c;
    c.dev = "/dev/ttyACM0"; c.vid = "10c4"; c.pid = "ea60"; c.serial = "SP001";
    c.driver = "cp210x"; c.product = "CP2102 USB to UART";
    std::vector<SerialPortInfo> ports{a, b, c};

    // ① 序列号（最可靠，即使不是第一个也能选中）
    auto m = io::match_serial_port(ports, "SP001", "", "");
    CHECK(m.dev == "/dev/ttyACM0" && m.rule == "serial_no", "规则①序列号命中 ttyACM0");

    // ② VID:PID（两个候选 ⇒ 取排序后第一个，稳定）
    m = io::match_serial_port(ports, "", "1a86:7523", "");
    CHECK(m.dev == "/dev/ttyUSB0" && m.rule == "vid_pid", "规则②VID:PID 命中 ttyUSB0（稳定取首个）");

    // ③ 子串（厂商/产品/驱动）
    m = io::match_serial_port(ports, "", "", "cp210x");
    CHECK(m.dev == "/dev/ttyACM0" && m.rule == "substr", "规则③驱动子串命中 ttyACM0");

    // ⭐ 优先级：序列号 > VID:PID > 子串
    m = io::match_serial_port(ports, "SP001", "1a86:7523", "ch341");
    CHECK(m.rule == "serial_no", "⭐ 优先级：serial_no 压过 vid_pid 与 substr");

    // ④ 唯一候选才自动选
    std::vector<SerialPortInfo> one{a};
    m = io::match_serial_port(one, "", "", "");
    CHECK(m.dev == "/dev/ttyUSB0" && m.rule == "unique", "规则④唯一候选自动选中");

    // ⑤ ⭐ 多候选 + 无规则命中 ⇒ 【不猜】，标记 ambiguous
    m = io::match_serial_port(ports, "", "", "");
    CHECK(m.dev.empty(), "⭐ 规则⑤多候选无命中 ⇒ 不选（dev 为空）");
    CHECK(m.ambiguous, "⭐ 且标记 ambiguous（提示用户列出候选）");

    // 空列表
    m = io::match_serial_port({}, "", "", "");
    CHECK(m.dev.empty() && !m.ambiguous, "无候选 ⇒ 不选，且不算 ambiguous");

    // vid_pid() 与 describe() 的健壮性
    CHECK(a.vid_pid() == "1a86:7523", "vid_pid() 拼接正确");
    SerialPortInfo empty;
    CHECK(empty.vid_pid().empty(), "空 VID/PID ⇒ vid_pid() 返回空串");
    CHECK(!c.describe().empty(), "describe() 非空");
  }

  std::printf("\n  %s\n", failures == 0 ? "全部通过" : "存在失败");
  return failures == 0 ? 0 : 1;
}
