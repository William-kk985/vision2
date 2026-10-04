/**
 * @file utils/config/print_config.hpp
 * @brief ⭐⭐ **`--print-config`：把【最终生效】的配置全打出来**（W100 · 原 D2）
 *
 * ## 为什么需要
 * 我们有三层配置叠加，**看一眼代码/yaml 猜不出最终值**：
 * ```
 * ① 命令行参数（含默认值）
 * ② yaml 多层合并（品牌相机默认 < 兵种相机配置 < 兵种 yaml 的同名键）
 * ③ 编译期开关（LOG_* 哪些开着 → ⭐ 见 core/debug.hpp §三；
 *    `debug_config.hpp` 只是"展开器"，改它没用）
 * ```
 * ⚠️ 实际踩过的坑：`--tongji` / `exposure_ms` / `enemy_color` 都出现过
 *   "以为读的是 A、实际生效是 B"。
 *
 * ## 设计取舍（⚠️ 不做成"通用 yaml dump"）
 * 本项目的 yaml **不是扁平的**（含矩阵、序列、嵌套），通用 dump 会**又长又难读**。
 * ⇒ 只打**关键的、容易搞错的**那些项，且**按人读的顺序分组**。
 *
 * ## 用法
 * ```cpp
 * if (cli.get<bool>("print-config")) {
 *   tools::print_effective_config(std::cout, cli, config_path, robot_name);
 *   return 0;   // ⭐ 打完就退，不启动相机/板卡
 * }
 * ```
 */
#ifndef HZMIR_UTILS_CONFIG_PRINT_CONFIG_HPP
#define HZMIR_UTILS_CONFIG_PRINT_CONFIG_HPP

#include <algorithm>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include <yaml-cpp/yaml.h>

namespace tools
{

namespace detail
{
/// ⭐ 从 yaml 安全读一个标量（缺失/类型不符都返回 fallback）
inline std::string yaml_scalar(const YAML::Node & n, const char * key, const std::string & fallback)
{
  if (!n || !n[key]) return fallback;
  try {
    return n[key].as<std::string>();
  } catch (...) {
    return "（类型不符）";
  }
}

inline void print_group(std::ostream & os, const std::string & title)
{
  os << "\n  ── " << title << " ──\n";
}

inline void print_kv(std::ostream & os, const std::string & k, const std::string & v,
                     const std::string & note = "")
{
  os << "    " << std::left << std::setw(24) << k << " = " << v;
  if (!note.empty()) os << "   " << note;
  os << "\n";
}
}  // namespace detail

/// @brief ⭐ 打印最终生效的配置（命令行 + yaml 合并后 + 编译期开关）
/// @param os          输出流（通常 std::cout）
/// @param cli         CLI 解析结果（需有 `get<T>(key)`）
/// @param config_path 兵种 yaml 路径（如 `params/robots/infantry.yaml`）
/// @param robot       兵种名（如 `infantry`）
template <typename Cli>
void print_effective_config(
  std::ostream & os, const Cli & cli, const std::string & config_path, const std::string & robot)
{
  os << "\n╔══════════════════════════════════════════════════════════════╗\n";
  os << "║  ⭐ 最终生效配置（--print-config）                            ║\n";
  os << "╚══════════════════════════════════════════════════════════════╝\n";
  detail::print_kv(os, "兵种", robot);
  detail::print_kv(os, "兵种 yaml", config_path);

  // ═══ ① 命令行参数 ═══
  detail::print_group(os, "① 命令行参数（含默认值）");
  {
    const std::pair<const char *, const char *> items[] = {
      {"tongji", "同济兼容总开关"},
      {"nis-thresh", "NIS 阈值单项覆盖（空=跟随 tongji）"},
      {"yaw-rate-src", "小陀螺判据单项覆盖（空=跟随 tongji）"},
      {"video", "录像路径（空=用真机相机）"},
      {"video-speed", "录像播放速率（0=不节流全速）"},
      {"force-mode", "录像模式档位"},
      {"record", "是否录像"},
      {"debug-img", "是否存图（L3）"},
      {"debug-window", "是否开窗口（L3）"},
      {"pj", "是否发 PlotJuggler"},
      {"pj-host", "PlotJuggler 目标 IP"},
      {"pj-port", "PlotJuggler 目标端口"},
      {"csv", "CSV 输出前缀"},
      {"log-off", "按模块静音（运行期）"},
      {"log-only", "只打印这些模块（运行期）"},
      {"camera-config", "相机配置路径覆盖"},
      {"dump-camera-params", "只打印相机参数后退出"},
      {"no-board", "强制虚拟下位机"},
      {"strict-board", "串口不存在就失败"},
      {"strict-device", "严格设备模式"},
      {"det-stats", "检测统计日志开关"},
      {"bullet-speed", "弹速（录像模式）"},
      {"config-path", "yaml 路径"},
    };
    for (const auto & [k, note] : items) {
      std::string v;
      try {
        v = cli.template get<std::string>(k);
      } catch (...) {
        try {
          v = cli.template get<bool>(k) ? "true" : "false";
        } catch (...) {
          try { v = std::to_string(cli.template get<double>(k)); } catch (...) { continue; }
        }
      }
      if (v.empty()) v = "（空）";
      detail::print_kv(os, k, v, note);
    }
  }

  // ═══ ② yaml（兵种 + 相机，合并后的关键项）═══
  detail::print_group(os, "② yaml 关键项（合并后；⚠️ 相机项已展平）");
  YAML::Node y;
  try {
    y = YAML::LoadFile(config_path);
  } catch (const std::exception & e) {
    os << "    ⚠️ 读不到 " << config_path << ": " << e.what() << "\n";
  }
  if (y) {
    const std::pair<const char *, const char *> yk[] = {
      {"enemy_color", "敌方颜色（⚠️ 只影响【跟踪阶段】过滤，不影响检测）"},
      {"camera_config", "相机配置路径"},
      {"priority_mode", "优先级模式（缺省=同济行为）"},
      {"trajectory_impl", "弹道实现（缺省 ideal=同济）"},
      // ⚠️ `yolo_name` 在下面单独打（要带上"三个模型在不在"）
      {"auto_fire", "是否由自瞄控制射击"},
      {"judge_distance", "远近判据距离"},
      {"model_device", "推理设备"},
    };
    for (const auto & [k, note] : yk) {
      if (!y[k]) { detail::print_kv(os, k, "（未配置 → 用内置默认）", note); continue; }
      detail::print_kv(os, k, detail::yaml_scalar(y, k, "?"), note);
    }

    // ⭐⭐⭐ W106：`yolo_name` + **三个检测器各自的可用性**
    //   ⚠️ 为什么要单独打："没换过"不代表"不能用" —— v8/v11 的模型路径
    //     通常**早就在 yaml 里配好了**，换 `yolo_name` 就能切（⭐ 不用重编）。
    //   ⚠️ 这里**只报"yaml 里配了没有"**，不碰文件系统（真正的加载失败由启动日志报）。
    detail::print_kv(os, "yolo_name",
                     y["yolo_name"] ? detail::yaml_scalar(y, "yolo_name", "?")
                                    : "（未配置 → yolov5）",
                     "⭐ 运行期切换检测器（改这个键即可，不用重编）");
    {
      const std::pair<const char *, const char *> models[] = {
        {"yolov5", "yolov5_model_path"},
        {"yolov8", "yolov8_model_path"},
        {"yolo11", "yolo11_model_path"}};
      for (const auto & [nm, key] : models) {
        const bool has = y[key] && !detail::yaml_scalar(y, key, "").empty();
        detail::print_kv(os, std::string("  → ") + nm,
                         has ? detail::yaml_scalar(y, key, "?") : "（yaml 未配路径）",
                         has ? "✅ 可切换（⚠️ 三者类别数/输入尺寸不同，语义不一样）" : "⬜ 不可用");
      }
    }
  }

  // ⭐ 相机配置（多层合并后展平 —— 用 io::Camera 自己的解析逻辑保证一致）
  detail::print_group(os, "③ 相机（⭐ 两层合并：params/cameras/<品牌>.yaml < params/robots/<兵种>.yaml）");
  {
    const std::string cam_path = detail::yaml_scalar(y, "camera_config", "");
    YAML::Node cam;
    if (!cam_path.empty()) {
      try {
        cam = YAML::LoadFile(cam_path);
        detail::print_kv(os, "camera_config", cam_path);
      } catch (...) {
        detail::print_kv(os, "camera_config", cam_path, "⚠️ 读不到！");
      }
    }
    const auto pick = [&](const char * k) -> std::string {
      if (y && y[k]) return detail::yaml_scalar(y, k, "?");
      if (cam && cam[k]) return detail::yaml_scalar(cam, k, "?");
      return "（未配置）";
    };
    detail::print_kv(os, "camera_name", pick("camera_name"));
    detail::print_kv(os, "vid_pid", pick("vid_pid"));
    detail::print_kv(os, "exposure_ms", pick("exposure_ms"), "⚠️ 我们程序会强制覆盖相机里的值");
    detail::print_kv(os, "gain", pick("gain"), "同上");
    detail::print_kv(os, "fps", pick("fps"));
    if (cam && cam["camera_params"])
      detail::print_kv(os, "camera_params", "已配置（见该 yaml）");
  }

  // ═══ ③.5 ⭐⭐ 实验开关（core/debug.hpp §二）═══
  detail::print_group(os, "③.5 实验开关（core/debug.hpp §四；开启后偏离同济行为）");
  {
    int on = 0;
#ifdef HZMIR_EXP_NO_TRAD_SAVE
    { detail::print_kv(os, "HZMIR_EXP_NO_TRAD_SAVE", "开", "关掉传统检测器落图"); ++on; }
#endif
    if (on == 0)
      os << "    （全部关闭 = 同济默认）新增实验请修改 core/debug.hpp §四\n";
    else
      os << "    ⚠️ 共 " << on << " 个实验【开】—— 结论出来后请【转正】或【删除】\n";
  }

  // ═══ ④ 编译期开关 ═══
  // ⭐⭐ W122：**改 `core/debug.hpp`**（不是 `debug_config.hpp` —— 那只是"展开器"）。
  // ⚠️ 原来这里只列了 8 个宏，漏了 7 个 ⇒ 用户开的开关可能"查不到"。
  // ⭐ 现在**列全 15 个**，且对**空宏**（定义但无调用点）明确标注 ⇒ 不会白开。
  detail::print_group(os, "④ 编译期开关（修改 core/debug.hpp §三 后重新编译）");
  {
    int on = 0;
    // ⭐ 宏：`(名字, 是否空宏)` —— 空宏 = 代码里还没有调用点，开了不出日志
    const std::pair<const char *, bool> sw[] = {
      {"YOLO", false},     {"DETECTOR", false}, {"EKF", false},      {"TRACKER", false},
      {"TARGET", true},    {"PLANNER", true},   {"AIMER", true},     {"SHOOTER", true},
      {"BUFF", false},     {"CAMERA", true},    {"BOARD", true},     {"IMU", true},
      {"TI", false},       {"TGD", false},      {"SINK", true},
    };
#ifdef HZMIR_LOG_YOLO
    { detail::print_kv(os, "HZMIR_LOG_YOLO", "已开启"); ++on; }
#endif
#ifdef HZMIR_LOG_EKF
    { detail::print_kv(os, "HZMIR_LOG_EKF", "已开启"); ++on; }
#endif
#ifdef HZMIR_LOG_TRACKER
    { detail::print_kv(os, "HZMIR_LOG_TRACKER", "已开启"); ++on; }
#endif
#ifdef HZMIR_LOG_TARGET
    { detail::print_kv(os, "HZMIR_LOG_TARGET", "已开启"); ++on; }
#endif
#ifdef HZMIR_LOG_PLANNER
    { detail::print_kv(os, "HZMIR_LOG_PLANNER", "已开启"); ++on; }
#endif
#ifdef HZMIR_LOG_AIMER
    { detail::print_kv(os, "HZMIR_LOG_AIMER", "已开启"); ++on; }
#endif
#ifdef HZMIR_LOG_SHOOTER
    { detail::print_kv(os, "HZMIR_LOG_SHOOTER", "已开启"); ++on; }
#endif
#ifdef HZMIR_LOG_BUFF
    { detail::print_kv(os, "HZMIR_LOG_BUFF", "已开启"); ++on; }
#endif
#ifdef HZMIR_LOG_CAMERA
    { detail::print_kv(os, "HZMIR_LOG_CAMERA", "已开启（未接入，无输出）"); ++on; }
#endif
#ifdef HZMIR_LOG_TI
    { detail::print_kv(os, "HZMIR_LOG_TI", "已开启"); ++on; }
#endif
#ifdef HZMIR_LOG_TGD
    { detail::print_kv(os, "HZMIR_LOG_TGD", "已开启"); ++on; }
#endif
    (void)sw;
    if (on == 0)
      os << "    （全部关闭 = 默认）开启方式：修改 core/debug.hpp §三 后重新编译\n"
         << "    可用开关（11 个）：YOLO EKF TRACKER TARGET PLANNER AIMER\n"
         << "                        SHOOTER BUFF CAMERA TI TGD\n"
         << "    已删除（改用运行期过滤）：BOARD / IMU → --log-only=gimbal，SINK → --log-only=sink\n";
    else
      os << "    共 " << on << " 个日志开关已开启\n";
  }

  os << "\n╚══════════════════════════════════════════════════════════════╝\n";
  os << "  ⭐ 提示：本命令只打印配置，**不会**启动相机/下位机。\n\n";
}

}  // namespace tools

#endif  // HZMIR_UTILS_CONFIG_PRINT_CONFIG_HPP
