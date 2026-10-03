/**
 * @file core/debug.hpp
 * @brief ⭐⭐ Debug **聚合层** —— 只做 `FrameDebug` 的组装
 *
 * ## W50（改进方案 A）：Debug 结构**跟角色走**了
 * 各角色的 Debug 快照**由角色自己拥有**，住在各自目录：
 * ```
 * core/auto_aim/detector/detector_debug.hpp     DetectorDebug
 * core/auto_aim/solver/solver_debug.hpp         SolverDebug
 * core/auto_aim/tracker/tracker_debug.hpp       TrackerDebug
 * core/auto_aim/target/target_debug.hpp         TargetDebug
 * core/auto_aim/planner/planner_debug.hpp       PlannerDebug
 * core/auto_aim/shooter/shooter_debug.hpp       ShooterDebug
 * core/auto_aim/controller/controller_debug.hpp ControllerDebug
 * core/auto_buff/buff_debug.hpp                 BuffDebug
 * ```
 * 本文件**只做两件事**：① include 它们 ② 定义 `FrameDebug` 聚合。
 *
 * ## ⭐ 依赖方向（单向，无环）
 * ```
 * <角色>/xxx_debug.hpp  →  core/debug.hpp  →  FrameDebug
 * ```
 * ⚠️ 角色头是**纯数据**，只依赖 `<cstdint>`，**不 include 本文件**。
 * （W50 之前本文件注释里说"会形成循环依赖" —— **那是错的**，已纠正。）
 *
 * ## ⚠️⚠️ 本文件【没有任何开关】—— 找开关请看别处（W103 补的指路牌）
 *
 * 名字里带 "debug" 的文件有好几个，**职责完全不同**，别找错：
 * ```
 * core/debug.hpp                    ← 你在这里：⭐【数据面】FrameDebug 的聚合（零宏）
 * utils/log/debug_config.hpp        ← ⭐⭐【日志开关总控】`HZMIR_LOG_*` / `LOG_*`
 * utils/log/log_filter.hpp          ← 【运行期】按模块过滤（--log-off / --log-only）
 * utils/debug/debug_sink.hpp        ← 【数据去向】SinkHub + IDebugSink
 * utils/debug/{csv,plotjuggler,window,image}_sink.hpp / recorder.hpp
 *                                      ← 各个 sink 的实现（热插拔的那些）
 * utils/config/{tongji_flags,stage_gate,print_config}.hpp
 *                                      ← 【入口开关】--tongji / --stop-after / --print-config
 * ```
 * ⭐ **想开某个模块的细节日志** ⇒ 改 `utils/log/debug_config.hpp` 里的一行（`#define` 取消注释）后重编。
 * ⭐ **想看当前生效的开关** ⇒ 跑 `--print-config`（第 ④ 节会列出哪些 `LOG_*` 开着）。
 *
 * ## 设计原则（doc 09 §14.1）
 *   ⭐ **L0~L2 常驻、零宏**（~ns 级，永远开）；**只有 L3 昂贵通道用宏**
 *
 * 级别：
 *   L0 耗时/帧号/模式      ~100 ns/帧   永远开
 *   L1 每角色一个快照结构  ~50 ns/帧    永远开
 *   L2 多帧序列（曲线）     ~50 ns/项    永远开（走 IDebugSink::on_series）
 *   L3 存图/显示            ~ms          ⭐ **纯运行期门控**（`SinkHub::wants_image()`，无宏）
 */
#ifndef HZMIR_CORE_DEBUG_NODE_HPP
#define HZMIR_CORE_DEBUG_NODE_HPP

#include <cstdint>

#include "core/types.hpp"

// ⭐⭐ W50：各角色的 Debug 快照（住在角色自己目录）
#include "core/auto_aim/controller/controller_debug.hpp"
#include "core/auto_aim/detector/detector_debug.hpp"
#include "core/auto_aim/planner/planner_debug.hpp"
#include "core/auto_aim/shooter/shooter_debug.hpp"
#include "core/auto_aim/solver/solver_debug.hpp"
#include "core/auto_aim/target/target_debug.hpp"
#include "core/auto_aim/tracker/tracker_debug.hpp"
#include "core/auto_buff/buff_debug.hpp"

namespace auto_aim
{

/// ⭐ 一帧的完整调试快照（L0 + L1）
struct FrameDebug
{
  // ── L0 常驻 ──
  int64_t t_frame_us = 0;
  // ⭐⭐ W86：**把「等相机」从 perceive 里拆出来**。
  //   原来 `t_perceive_us` 含 `camera->read()` 的**阻塞等待** —— 真机 30fps 下
  //   那一段约 **28 ms**（= 33.3 ms 帧周期 − 5.3 ms 推理），看起来像"perceive 极贵"，
  //   实际是**在等相机**，不耗 CPU。拆开后：
  //     · `t_cam_wait_us` = 阻塞等帧（不耗 CPU，等于帧率上限）
  //     · `t_perceive_us` = 真正的活（board 读 + recorder 入队 + set_R_gimbal2world）
  int64_t t_cam_wait_us = 0;
  int64_t t_perceive_us = 0;
  int64_t t_decide_us = 0;
  uint32_t frame_id = 0;
  uint8_t mode = 0;
  uint8_t game_state = 0;

  // ── L1 快照（⭐ 各角色自带，定义在各自目录）──
  DetectorDebug detector;
  SolverDebug solver;
  TrackerDebug tracker;
  TargetDebug target;
  PlannerDebug planner;
  ShooterDebug shooter;
  ControllerDebug controller;
  BuffDebug buff;
};

}  // namespace auto_aim

#endif  // HZMIR_CORE_DEBUG_NODE_HPP
