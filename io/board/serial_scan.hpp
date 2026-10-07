/**
 * @file  io/board/serial_scan.hpp
 * @brief 串口设备扫描与自适应选择（哨兵串口用；也可供其他串口链路复用）
 *
 * ## 为什么需要
 * 下位机的串口设备名**不稳定**：
 *   · USB 转串口按插入顺序枚举 ⇒ 今天 `/dev/ttyUSB0`、明天可能是 `/dev/ttyUSB1`
 *   · 换了 USB 口，或同时插了调试线与下位机线 ⇒ 名字会变
 *   · udev 规则能固定名字（⭐ 推荐做法），但需要系统配置，换机器就没了
 *
 * ## ⚠️ 原则：**确定性优先，绝不静默选错设备**
 * 选错串口比选不到更糟——会往错误的设备写数据，且**不会报错**。
 * 因此只在**明确要求自适应**（`serial_port` 留空或写 `auto`）时才扫描；
 * 若 yaml 给了具体路径，即使设备不存在也**只报错、不自动换**。
 *
 * ## 分层策略
 * | 情形 | 行为 |
 * |---|---|
 * | yaml 给具体路径且设备存在 | ⭐ 直接用（比赛要确定性） |
 * | yaml 给具体路径但设备不存在 | ⚠️ 报错 + 列出可用串口（**不自动换**） |
 * | yaml 为空 / 写 `"auto"` | ⭐ 自适应扫描（见下） |
 *
 * ## 自适应扫描规则（按可靠性降序）
 * | 规则 | 依据 | 可靠性 |
 * |---|---|---|
 * | ① 序列号 | `/sys/class/tty/<dev>/device/serial` | ⭐⭐⭐ 最可靠（需下位机烧录固定序列号） |
 * | ② VID:PID | `/sys/class/tty/<dev>/device/idVendor` + `idProduct` | ⭐⭐ 可靠（yaml 可配） |
 * | ③ 厂商 / 产品串 | `.../manufacturer` / `product` | ⭐ 一般 |
 * | ④ 唯一候选 | 只有一个串口 | ⭐ 可用（多设备时不生效） |
 * | ⑤ 多个候选且无规则命中 | — | ⚠️ **报错列出全部，不猜** |
 *
 * ## yaml 配置
 * ```yaml
 * serial_port: "auto"        # 或留空；给具体路径则严格使用该路径
 * serial_serial_no: "SP001"  # 可选：匹配 /sys/.../serial
 * serial_vid_pid: "1a86:7523" # 可选：匹配 idVendor:idProduct（CH340）
 * serial_match: "ch341"      # 可选：在 manufacturer/product/driver 里做子串匹配
 * ```
 *
 * ## 本机限制
 * ⚠️ 开发机（无 USB 串口）上 `scan_serial_ports()` 返回空表 ⇒ 自适应会失败并给出提示。
 *    这是**预期行为**，不是 bug。
 */
#ifndef HZMIR_IO_BOARD_SERIAL_SCAN_HPP
#define HZMIR_IO_BOARD_SERIAL_SCAN_HPP

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <optional>
#include <string>
#include <system_error>
#include <vector>

namespace io
{

/// @brief 扫描到的一个串口设备
struct SerialPortInfo
{
  std::string dev;           ///< 设备节点，如 `/dev/ttyUSB0`
  std::string vid;           ///< USB VID（可能是空串，非 USB 串口）
  std::string pid;           ///< USB PID
  std::string serial;        ///< 序列号（需设备烧录；可能为空）
  std::string manufacturer;  ///< 厂商串
  std::string product;       ///< 产品串
  std::string driver;        ///< 驱动名（ch341 / cp210x / ftdi_sio …）

  /// @brief `VID:PID`（如 `1a86:7523`）；任一为空则返回空串
  std::string vid_pid() const
  {
    if (vid.empty() || pid.empty()) return {};
    return vid + ":" + pid;
  }

  /// @brief 供日志/提示用的单行描述
  std::string describe() const
  {
    std::string s = dev;
    if (!vid_pid().empty()) s += " [" + vid_pid() + "]";
    if (!product.empty()) s += " " + product;
    else if (!manufacturer.empty()) s += " " + manufacturer;
    if (!serial.empty()) s += " sn=" + serial;
    if (!driver.empty()) s += " drv=" + driver;
    return s;
  }
};

namespace detail
{

/// @brief 读一个 sysfs 文本属性（可能不存在 ⇒ 返回空串；**不抛异常**）
inline std::string read_sys_attr(const std::filesystem::path & p)
{
  std::ifstream f(p);
  if (!f) return {};
  std::string s;
  std::getline(f, s);
  // 去掉首尾空白与换行
  const auto b = s.find_first_not_of(" \t\r\n");
  if (b == std::string::npos) return {};
  const auto e = s.find_last_not_of(" \t\r\n");
  return s.substr(b, e - b + 1);
}

/// @brief 判断文件名是否像串口设备
inline bool looks_like_tty(const std::string & name)
{
  return name.rfind("ttyUSB", 0) == 0 || name.rfind("ttyACM", 0) == 0 ||
         name.rfind("ttyS", 0) == 0;
}

}  // namespace detail

/// @brief 扫描 `/dev` 下的串口设备，并尽量补全 sysfs 属性
/// @note 按设备名排序 ⇒ 结果稳定（便于日志对比与单测）
inline std::vector<SerialPortInfo> scan_serial_ports()
{
  std::vector<SerialPortInfo> out;
  std::error_code ec;
  const std::filesystem::path dev_dir{"/dev"};
  if (!std::filesystem::exists(dev_dir, ec)) return out;

  for (const auto & e : std::filesystem::directory_iterator(dev_dir, ec)) {
    const auto name = e.path().filename().string();
    if (!detail::looks_like_tty(name)) continue;

    SerialPortInfo info;
    info.dev = e.path().string();

    // ⭐ /sys/class/tty/<name>/device/ 下是 USB 属性（非 USB 串口可能没有该目录）
    const std::filesystem::path base = std::filesystem::path("/sys/class/tty") / name / "device";
    info.vid = detail::read_sys_attr(base / "idVendor");
    info.pid = detail::read_sys_attr(base / "idProduct");
    info.serial = detail::read_sys_attr(base / "serial");
    info.manufacturer = detail::read_sys_attr(base / "manufacturer");
    info.product = detail::read_sys_attr(base / "product");

    // 驱动名在 device/driver 这个符号链接的目标名上
    std::error_code ec2;
    const auto drv = std::filesystem::read_symlink(base / "driver", ec2);
    if (!ec2 && !drv.empty()) info.driver = drv.filename().string();

    out.push_back(std::move(info));
  }

  std::sort(out.begin(), out.end(), [](const auto & a, const auto & b) { return a.dev < b.dev; });
  return out;
}

/// @brief 自适应匹配的结果
struct SerialMatchResult
{
  /// 匹配到的设备（空 = 没匹配上）
  std::string dev;
  /// 用了哪条规则（日志/诊断用，如 `"serial_no"` / `"vid_pid"` / `"unique"` / `"none"`）
  std::string rule;
  /// 是否因"多个候选且无规则命中"而放弃（此时 `dev` 为空，需向用户列出候选）
  bool ambiguous = false;
};

/// @brief 在候选列表里按规则挑一个设备
/// @param ports     `scan_serial_ports()` 的结果
/// @param serial_no 期望序列号（空 = 不按此规则）
/// @param vid_pid   期望 `VID:PID`（空 = 不按此规则）
/// @param substr    在 manufacturer/product/driver 里做子串匹配（空 = 不按此规则）
/// @note ⭐ **规则按可靠性降序尝试**；都不命中时：唯一候选才自动选，否则标 `ambiguous`。
inline SerialMatchResult match_serial_port(
  const std::vector<SerialPortInfo> & ports, const std::string & serial_no,
  const std::string & vid_pid, const std::string & substr)
{
  // ① 序列号（最可靠）
  if (!serial_no.empty()) {
    for (const auto & p : ports)
      if (p.serial == serial_no) return {p.dev, "serial_no", false};
  }
  // ② VID:PID
  if (!vid_pid.empty()) {
    for (const auto & p : ports)
      if (p.vid_pid() == vid_pid) return {p.dev, "vid_pid", false};
  }
  // ③ 厂商/产品/驱动子串
  if (!substr.empty()) {
    for (const auto & p : ports) {
      const bool hit = p.manufacturer.find(substr) != std::string::npos ||
                       p.product.find(substr) != std::string::npos ||
                       p.driver.find(substr) != std::string::npos;
      if (hit) return {p.dev, "substr", false};
    }
  }
  // ④ 唯一候选
  if (ports.size() == 1) return {ports.front().dev, "unique", false};
  // ⑤ 无规则命中且候选不唯一 ⇒ ⭐ 不猜
  return {{}, "none", !ports.empty()};
}

}  // namespace io

#endif  // HZMIR_IO_BOARD_SERIAL_SCAN_HPP
