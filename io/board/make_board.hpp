/**
 * @file io/board/make_board.hpp
 * @brief ⭐⭐⭐ **下位机自动感应**（W54）—— 4 个兵种共用，避免重复
 *
 * ## 用户场景
 * 「**这台电脑只插了摄像头，想要尝试一下**」——
 * 原来没串口就 `exit(1)`，摄像头能用却跑不起来。
 *
 * ## 决策表
 * | `--no-board` | `com_port` 存在 | `--strict-board` | 结果 |
 * |---|---|---|---|
 * | 是 | — | — | ⭐ `VirtualBoard`（显式要求无板） |
 * | 否 | ✅ | — | `Gimbal`（真下位机） |
 * | 否 | ❌ | 否 | ⭐ **`VirtualBoard` + 大声警告**（默认降级） |
 * | 否 | ❌ | **是** | ❌ **报错退出**（同济行为：没下位机就失败） |
 *
 * ⭐ **默认降级**的理由：没接硬件是**开发机的常态**，直接崩体验太差；
 *   而**比赛**上要的是"宁可失败也不要静默跑错" → 那时用 `--strict-board`。
 */
#ifndef HZMIR_IO_BOARD_MAKE_BOARD_HPP
#define HZMIR_IO_BOARD_MAKE_BOARD_HPP

#include <filesystem>
#include <memory>
#include <string>

#include "io/board/board.hpp"
#include "io/board/gimbal/gimbal.hpp"
#include "io/board/virtual_board.hpp"
#include "utils/log/logger.hpp"
#include "utils/yaml/yaml.hpp"

namespace io
{

/// @brief 读配置里的 `com_port`
inline std::string configured_port(const std::string & config_path)
{
  try {
    return tools::read<std::string>(tools::load(config_path), "com_port");
  } catch (const std::exception &) {
    return {};   // 没有这个字段（如 sentry/uav 的 yaml）→ 空
  }
}

/**
 * @brief ⭐⭐ 按「自动感应」结果造下位机
 * @param no_board   `--no-board`：强制虚拟板
 * @param strict     `--strict-board`：串口不存在就**报错退出**（同济行为）
 * @param bullet_speed 虚拟板上报的弹速（真板子从串口拿）
 * @param force_mode 虚拟板上报的档位（-1 = 默认 AUTO_AIM）
 * @param robot      日志前缀（兵种名）
 * @return `IBoard`；`strict` 且串口缺失时**不返回**（直接 exit）
 */
inline std::unique_ptr<IBoard> make_board(
  const std::string & config_path, bool no_board, bool strict, double bullet_speed, int force_mode,
  const std::string & robot)
{
  const auto port = configured_port(config_path);
  const bool have_port = !port.empty() && std::filesystem::exists(port);

  const auto fm = (force_mode >= 0 && force_mode <= 3) ? static_cast<GimbalMode>(force_mode)
                                                       : GimbalMode::AUTO_AIM;

  // ── ① 显式要求无板 ──
  if (no_board) {
    tools::logger()->info("[{}] `--no-board` → 使用虚拟下位机（不碰串口）", robot);
    return std::make_unique<VirtualBoard>(fm, bullet_speed, "--no-board 显式指定");
  }

  // ── ② 串口在 ──
  if (have_port) {
    tools::logger()->info("[{}] 真实硬件模式（下位机 {}）", robot, port);
    return std::make_unique<Gimbal>(config_path);
  }

  // ── ③ ⚠️ 串口不在 ──
  std::string why = port.empty() ? "参数文件里**没有 `com_port`** 字段"
                                 : ("串口 '" + port + "' **不存在**");

  if (strict) {
    tools::logger()->error(
      "[{}] `--strict-board` 且{} → 失败退出（同济行为）。\n"
      "  想跑起来: 去掉 `--strict-board`（自动降级虚拟板），或 `--no-board`，或 `--video=录像.avi`",
      robot, why);
    std::exit(1);
  }

  tools::logger()->warn("[{}] {} → 自动降级为虚拟下位机", robot, why);
  return std::make_unique<VirtualBoard>(fm, bullet_speed, why);
}

}  // namespace io

#endif  // HZMIR_IO_BOARD_MAKE_BOARD_HPP
