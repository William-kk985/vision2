/**
 * @file core/debug.hpp
 * @brief ⭐⭐⭐ **开关与实验总控** —— 全项目的「编译期开关」都在这一个文件里
 *
 * ⭐ **怎么用（3 步）**
 * ```bash
 * ① 在【§三 日志开关】或【§四 实验层】里，把某行的 `//` 去掉
 * ② 重新编译：  cmake --build exp/hzmir_build -j$(nproc)
 * ③ 跑起来看：  tools/scripts/run.sh infantry --video=xxx.avi
 * ```
 * ⭐ **开之前先问：能不能【不重编】就看到？**
 * ```bash
 * --log-only=<模块>       ⭐ 运行期过滤日志（大小写不敏感 + 前缀匹配），不用重编
 * --print-config          看当前配置项（含本文件里哪些开关是开的）
 * --csv=xxx / 热键 3      ⭐ 看【数值趋势】（PlotJuggler）
 * 热键 d / --log-level    调运行期日志级别
 * ```
 * ⇒ ⭐ **本文件的宏只留给【真正的逐帧细节】**。默认**全关**。
 *
 * ## ⚠️ 别搞混：名字像 debug 的文件，职责完全不同
 * ```
 * core/debug.hpp              ← ⭐【你在这里】开关与实验总控（纯宏、**零依赖**）
 * core/debug_node.hpp         ← 【数据面】FrameDebug 聚合 8 个角色的快照（零宏）
 * utils/log/debug_config.hpp  ← 【展开器】把本文件的开关翻译成 LOG_xxx(...) 宏
 * utils/log/log_filter.hpp    ← 【运行期】按模块过滤（--log-off / --log-only）
 * utils/debug/debug_sink.hpp  ← 【数据去向】SinkHub + IDebugSink
 * utils/debug/*_sink.hpp      ← 各 sink 实现（热插拔的那些）
 * ```
 *
 * ## ⭐ 三层「测试/调试」策略（本文件是中间那层）
 * ```
 * ┌─ 正式代码（core/…/<角色>/）─── 长期维护，**以同济为准**
 * ├─ ⭐⭐ 本文件（宏 + 重编译）───── **中间层**：算法的【临时添加 / 替换】
 * └─ test/ ─────────────────────── 单一测试 / 较大的改动测试
 * ```
 * ⭐ **为什么要有中间层**：直接在正式代码里手改做实验**很容易忘改回来**
 *   ⇒ 放这儿 = **一行开关，可发现、可一键还原**。
 *
 * ⭐ **为什么用宏而不是运行期热插拔**：最终只会留一套算法 ⇒ 不需要"装着两套随时切"；
 *   而热插拔会让"当前跑的是哪套"**不确定**，算法对比最怕这个。
 *   ⚠️ 但**兼容性**保留：开关默认全关（= 完全同济行为）。
 *   📖 完整理由 + 宏纪律 ⇒ `docs/16-调试体系设计与可融入项.md`
 */
#ifndef HZMIR_CORE_DEBUG_HPP
#define HZMIR_CORE_DEBUG_HPP

// ═══════════════════════════════════════════════════════════════════════════
// §一 ⭐⭐ 总览：16 个开关一览（先看这张表，再决定开哪个）
//
// ⚠️⚠️ **先看【状态】列**（W123 更新）：
//   · ✅ **有效** —— 打开就有日志（**11 个**）
//   · ⚠️ **未接** —— 宏在、但代码里没有调用点，**打开后什么都不会发生**（1 个：`CAMERA`）
//   · 🗑️ **冗余** —— ⚠️ **它的模块【已经有 info/debug 日志】了**，再加宏只会重复
//     （`BOARD` 的 `gimbal.cpp` 有 **19 处**、`SINK` 的生命周期有 **4 处 info**、
//      `IMU` 与 `BOARD` 同文件且边界不清）⇒ ⭐ **别用这三个，用 `--log-only=` 过滤现成的**
//   ⭐ **实际可用的 11 个**：`YOLO` `DETECTOR` `EKF` `TRACKER` `TARGET` `PLANNER`
//     `AIMER` `SHOOTER` `BUFF` `TI` `TGD`
//
// | 开关 | 状态 | 看什么 | 吵不吵 |
// |---|---|---|---|
// | ⭐ **HZMIR_LOG_YOLO** | ✅ | YOLO 逐帧细节（推理耗时 / 两道门槛 / 完整漏斗 / 峰值位置） | ⚠️⚠️ 每帧 2 行 |
// | **HZMIR_LOG_DETECTOR** | ✅ | 传统检测器：灯条 / 装甲板**配对**细节 | ⚠️⚠️ 每帧多行 |
// | **HZMIR_LOG_EKF** | ✅ | EKF 新息（NIS）/ 收敛 / 发散判定 | ⚠️⚠️⚠️ **最吵** |
// | **HZMIR_LOG_TRACKER** | ✅ | 跟踪**状态机切换**（lost/detecting/tracking/temp_lost） | ⭐ 低频 |
// | **HZMIR_LOG_TARGET** | ✅ | 目标选择 / 跳变 / **小陀螺判据** | ⭐ 中频 |
// | **HZMIR_LOG_PLANNER** | ✅ | MPC **迭代次数** / 弹道 / 重合度 | ⭐ 中频 |
// | **HZMIR_LOG_AIMER** | ✅ | 瞄点选择 / **延迟补偿** | ⭐ 中频 |
// | ⭐ **HZMIR_LOG_SHOOTER** | ⚠️ **空** | **开火判据为什么没过** | ⭐ 中频 |
// | **HZMIR_LOG_BUFF** | ✅ | 打符：检测 / 拟合 / 预测 | ⭐ 中频 |
// | **HZMIR_LOG_CAMERA** | ⚠️ **未接** | 相机 SDK 参数 / 带宽 / **丢帧** | ⭐ 低频 |
// | **HZMIR_LOG_BOARD** | 🗑️ **冗余** | 下位机**收发** | ⚠️⚠️⚠️ **每帧都发，极吵** |
// | **HZMIR_LOG_IMU** | 🗑️ **冗余** | IMU 数据 / **时间戳对齐** | ⚠️ 高频 |
// | **HZMIR_LOG_TI** | ✅ | 时序积分器（`TemporalIntegrator`） | ⭐ 低频 |
// | **HZMIR_LOG_TGD** | ✅ | 传统检测的 **TGD**（目标引导检测） | ⭐ 中频 |
// | **HZMIR_LOG_SINK** | 🗑️ **冗余** | sink 生命周期（挂上 / 摘掉 / 队列深度） | ⭐ 低频 |
// | ⭐ **HZMIR_EXP_NO_TRAD_SAVE** | ✅ | **关掉**传统检测器的落图（跑批调参省盘） | — |
//
// ⭐ **"关掉 = 零开销"是真的**：宏展开成 `LOG_xxx(...)`，关掉时**整条语句连参数一起消失**
//   （实测 **1.65 ns/次 vs 运行期过滤 89 ns/次，快 54×**）⇒ 含昂贵参数的日志收益最大。
// ⭐ **每个宏都支持 `--log-only=<模块名>` 运行期过滤**（不用重编）。
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// §二 ⭐⭐ 按场景找开关（"我现在想查 X，该开哪个？"）
//
// ── A：识别率不行 / 检测不到装甲板 ──
//   ① ⭐ **先别开宏** —— 概览日志**运行期**就能看（不用重编）：
//        tools/scripts/run.sh infantry --log-only=yolo --video=xxx.avi
//      会看到两类（**已限频**，不刷屏）：
//        "本帧无候选：objectness 峰值 X < 阈值 Y"（每 60 帧一条）
//        "objectness 通过 N → 输出 M（滤掉 not_armor/conf/type）"（每 30 帧一条）
//   ② 还看不清 ⇒ 开 **`HZMIR_LOG_YOLO`**（⚠️ 每帧 2 行，但信息最全）
//   ③ 传统检测器（`detector_impl=traditional`）⇒ 开 `HZMIR_LOG_DETECTOR`
//   ④ ⚠️ **"检测到的是蓝的但 tracker 不收"** ⇒ 是**敌我颜色**问题，**不是日志问题**：
//        tools/scripts/run.sh infantry --enemy-color=blue ...
//      （敌我颜色**只影响跟踪阶段**，不影响检测 —— 见 docs/19）
//
// ── B：EKF 状态不对 / 目标乱跳 / 小陀螺打不准 ──
//   ① 先开 **`HZMIR_LOG_TRACKER`**（低频）—— 看状态机有没有正常 `tracking`
//   ② 再开 **`HZMIR_LOG_EKF`** ✅（⚠️⚠️⚠️ 最吵，建议配 `--log-only=ekf`）
//   ③ 目标选择问题 ⇒ `HZMIR_LOG_TARGET` ✅（发散原因 + 首次收敛）
//
// ── C：能跟踪但打不中 / 不开火 ──
//   ① ⭐ **`HZMIR_LOG_SHOOTER`** ✅ —— 开火判据为什么没过（报 traj_err/thresh/弹速/飞行/距离）
//      （判据 = `traj_err < fire_thresh`，默认 0.003，见 `planner/mpc.cpp`）
//   ② `HZMIR_LOG_PLANNER` ✅（迭代/重合度/acc_max）③ `HZMIR_LOG_AIMER` ✅（瞄点/弹道无解）
//   ④ ⭐ **云台不动？不用开宏** —— 看有没有这条 **info**：
//        `[Board] first control command: fire=... yaw=...`
//      长时间没有 ⇒ **没走到发送**（W119 新增，一处覆盖四兵种）
//
// ── D：打符 ⇒ `HZMIR_LOG_BUFF` ✅ ──
// ── E：硬件层 ⇒ ⭐ **不用开宏**：`io/board/gimbal/gimbal.cpp` 已有 **19 处**日志，
//        用 `--log-only=gimbal` 过滤即可（`CAMERA` 宏暂未接；`BOARD`/`IMU` 是冗余）──
// ── F：想知道 sink（存图/窗口/CSV）挂上没 ⇒ ⭐ **不用开宏**：
//        `[ImageSink]` / `[WindowSink]` 的生命周期**已经有 info 日志**（`--log-only=sink`）
//        或者直接看启动时的 `[ImageSink] -> output/images ...` info ──
// ── G：跑批调参，想省盘 ⇒ ⭐ **`HZMIR_EXP_NO_TRAD_SAVE`** ──
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// §三 ⭐⭐ 日志开关 —— **这一节是你实际要改的地方**（`#define` = 开，注释掉 = 关）
//
// ⚠️ 展开成 `LOG_xxx(...)` 的地方在 `utils/log/debug_config.hpp`（本文件只声明开关）。
// ⭐ 建议：【关】每帧刷屏的细节；【开】低频的状态变化。
// ⚠️ 15 个宏都**默认关**。
// ⭐ **W123 后能出日志的 11 个**：`YOLO` `DETECTOR` `EKF` `TRACKER` `TARGET` `PLANNER`
//    `AIMER` `SHOOTER` `BUFF` `TI` `TGD`
// ⚠️ **`CAMERA` 未接**（宏在、无调用点）⇒ ⭐ **别白开**
// 🗑️ **`BOARD` `IMU` `SINK` 是冗余**（模块已有现成日志）⇒ ⭐ **改用 `--log-only=`**：
//      `--log-only=gimbal`（19 处）· `--log-only=sink`（生命周期 4 处 info）
// ═══════════════════════════════════════════════════════════════════════════

// ── 检测链路 ──
// #define HZMIR_LOG_YOLO            // ⭐ YOLO 逐帧细节（⚠️ 每帧 2 行）—— 含：
//   ① `infer()` 单行耗时（⚠️ 与 CSV 的 `det_t_infer_us` 不同：那是【整段 detect】）
//   ② **两道门槛**：`obj 0.70 → conf 0.80`（+NMS）⚠️ 概览只报第一道，不说清第二道会误判
//   ③ **完整漏斗**：`25200 anchor → >0.5 → >0.3 → >0.1 → pass → NMS → 输出`
//   ④ **峰值在画面哪里**（归一化坐标）⭐ 区分「中心 ⇒ 图像质量」vs「边缘 ⇒ 背景误检」
//   ⑤ 被 `check_type` 丢掉的名字（`base×1` 这种）
// #define HZMIR_LOG_DETECTOR        // 传统检测器的灯条/装甲板配对细节

// ── 跟踪 / 估计 ──
// #define HZMIR_LOG_EKF             // ⚠️⚠️⚠️ 最吵：EKF 新息（NIS）/ 收敛 / 发散判定
// #define HZMIR_LOG_TRACKER         // ⭐ 低频：跟踪状态机切换（lost/detecting/tracking/temp_lost）
// #define HZMIR_LOG_TARGET          // ✅ 目标选择 / 跳变 / 小陀螺判据 / 发散原因 / 首次收敛

// ── 规划 / 射击 ──
// #define HZMIR_LOG_PLANNER         // ✅ MPC 迭代 / 重合度 / acc_max / 弹道
// #define HZMIR_LOG_AIMER           // ✅ 瞄点选择 / 弹道无解
// #define HZMIR_LOG_SHOOTER         // ✅ 开火判据为什么没过（traj_err/thresh/弹速/飞行/距离）

// ── 打符 ──
// #define HZMIR_LOG_BUFF            // 打符：检测 / 拟合 / 预测

// ── 硬件 / 板卡 ──
// #define HZMIR_LOG_CAMERA          // ⚠️ 未接（无调用点）· 相机 SDK 参数 / 带宽 / 丢帧
// #define HZMIR_LOG_BOARD           // 🗑️ 冗余（gimbal.cpp 已有 19 处）· 用 --log-only=gimbal
// #define HZMIR_LOG_IMU             // 🗑️ 冗余（与 BOARD 同文件）· 用 --log-only=gimbal

// ── 轮子（`utils/wheels/`）──
// #define HZMIR_LOG_TI              // 时序积分器（TemporalIntegrator）
// #define HZMIR_LOG_TGD             // 传统检测的 TGD（目标引导检测）

// ── 调试体系自身 ──
// #define HZMIR_LOG_SINK            // 🗑️ 冗余（生命周期已有 4 处 info）· 用 --log-only=sink

// ═══════════════════════════════════════════════════════════════════════════
// §四 ⭐⭐ 实验层 —— 「临时添加 / 替换算法」用这一层
//
// ## 加一个新实验（三步）
//   ① 实现放 **`test/function/exp_<名字>.{hpp,cpp}`**（⭐ 一眼看出是实验）
//   ② 在下面加一行 `#define HZMIR_EXP_<名字>`
//   ③ 在**相应文件**加 `#ifdef` 入口（⚠️ 装配点，不许散落到算法内部）
//      ── ⭐ **CMake 自动解析本文件**：定义了哪个 `HZMIR_EXP_*`，就自动
//         ① 全项目定义该宏 ② 把对应的 `exp_*.cpp` 加进构建 ⇒ **不用手改 CMakeLists**
//
// ## ⚠️⚠️ 生命周期（"中间层"的关键）
//   实验**验证完必须二选一**：**转正**（搬进 `core/…/<角色>/`，删开关和 `#ifdef`）
//   或 **放弃**（删 `exp_*.cpp` + 开关 + `#ifdef`）。
//   ⚠️ **不许长期挂在这儿** —— 否则它就从"实验"变成"没人敢删的隐藏功能"。
//
// ## ⭐ 关掉时是【真·零残留】
//   `exp_*.cpp` **不参与编译**（CMake 层面就不加），`#ifdef` 里的代码**不存在**
//   ⇒ 没有"编进去了但没跑"的死代码，也没有运行期分支。
//
// 📖 **完整模板（含 `init()` / 装配点写法）** ⇒ `docs/16-调试体系设计与可融入项.md`
// ═══════════════════════════════════════════════════════════════════════════

// ── 实验：传统检测器的落图 ──
// ⚠️ 背景：`Detector::save()` 目前**无条件**落图（`detector.cpp` 的 check_name/check_type）
//   —— 置信度不过/类型异常就落一张 JPEG（"用于分类器迭代"）。
//   ⚠️ 而**三个 YOLO 版本的同类调用是注释掉的** ⇒ 两边不一致。
//   ⭐ 默认**保持同济行为（= 落图）**；开了这个才**关掉**（跑批省盘 + 省时间）。
// #define HZMIR_EXP_NO_TRAD_SAVE    // ⭐ 关掉传统检测器的落图（跑批调参时开）

// ═══════════════════════════════════════════════════════════════════════════
// §五 兼容开关（编译期）
//
// ⚠️ 与 §四 的区别：§四 是「临时实验」，§五 是「**长期的兼容/回退路径**」（可长期存在）。
// ⭐ 现状：本项目在**源码里硬编码**的偏差，由 **`--tongji`（运行期）** 集中控制
//   （见 `utils/config/tongji_flags.hpp`），所以这里**暂时是空的**。
// ═══════════════════════════════════════════════════════════════════════════

#endif  // HZMIR_CORE_DEBUG_HPP
