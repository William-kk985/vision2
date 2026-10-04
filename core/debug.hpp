/**
 * @file  core/debug.hpp
 * @brief 编译期开关总控（日志开关与算法实验开关）
 *
 * 本文件是全项目**编译期开关**的唯一入口。所有开关默认关闭，等同于同济原始行为。
 *
 * @section usage 使用方式
 * @code{.sh}
 * # 1. 在 §三 或 §四 中去掉目标开关行首的 "//"
 * # 2. 重新编译
 * cmake --build exp/hzmir_build -j$(nproc)
 * # 3. 运行并观察输出
 * tools/scripts/run.sh infantry --video=assets/demo/demo.avi --force-mode=1
 * @endcode
 *
 * 查询当前生效的开关：
 * @code{.sh}
 * ./src/infantry --print-config params/robots/infantry.yaml
 * @endcode
 *
 * @section runtime_first 优先使用运行期手段
 * 下列需求无需重新编译即可满足，应先于本文件的开关使用：
 *
 * | 手段 | 用途 |
 * |---|---|
 * | `--log-only=<模块>` | 运行期日志过滤；大小写不敏感，支持前缀匹配 |
 * | `--print-config` | 打印全部生效配置项，含本文件各开关状态 |
 * | `--csv=<前缀>` / 热键 `3` | 输出逐帧数值，供 PlotJuggler 分析 |
 * | 热键 `d` / `--log-level` | 调整运行期日志级别 |
 *
 * 本文件的宏仅服务于**逐帧细节**这一层次，关闭时无任何运行开销。
 *
 * @section related 相关文件
 * 以下文件名均含 "debug"，但职责互不相同：
 *
 * | 文件 | 职责 |
 * |---|---|
 * | `core/debug.hpp` | 本文件。开关声明，纯宏、零依赖 |
 * | `core/debug_node.hpp` | 数据面。`FrameDebug` 聚合各角色快照，不含宏 |
 * | `utils/log/debug_config.hpp` | 展开器。将本文件开关展开为 `LOG_xxx(...)` 宏 |
 * | `utils/log/log_filter.hpp` | 运行期按模块过滤日志 |
 * | `utils/debug/debug_sink.hpp` | 数据去向。`SinkHub` 与 `IDebugSink` |
 * | `utils/debug/*_sink.hpp` | 各 sink 实现 |
 *
 * @section layering 三层测试与调试策略
 * @verbatim
 * 正式代码 core/<角色>/     长期维护，以同济实现为准
 * 本文件（宏 + 重新编译）   中间层，用于算法的临时添加与替换
 * test/                     单元测试与较大改动的验证
 * @endverbatim
 *
 * 中间层的意义：直接在正式代码中改动做实验容易遗漏还原，而开关集中于此，
 * 可被检索、可一次性还原。
 *
 * @section why_macro 为何采用编译期宏而非运行期热插拔
 * - 算法最终只保留一套实现，无需在同一二进制中常驻两套；
 * - 运行期切换会使"当前运行的是哪一套"不确定，不利于算法对比。
 *
 * 但兼容性必须保留：开关默认关闭，等同于同济行为。既有的运行期槽位
 * （`trajectory_impl`、`detector_impl` 等）属于配置而非实验，继续保留。
 *
 * @section macro_discipline 宏纪律
 * 宏仅承担三项职责：是否编译、默认值、互斥检查。`#ifdef` 只允许出现在装配点
 * （角色的构造或工厂处），不得散落至算法实现文件中。
 */
#ifndef HZMIR_CORE_DEBUG_HPP
#define HZMIR_CORE_DEBUG_HPP

// ═══════════════════════════════════════════════════════════════════════════
// §一  开关总览
//
// 状态说明：
//   ✅ 可用    —— 打开后即产生日志
//   ⚠️ 未接入  —— 宏已定义但代码中无调用点，打开后不产生输出
//
// 开销说明：宏关闭时整条语句连同参数一并消失，实测 1.65 ns/次；
// 运行期过滤为 89 ns/次。含昂贵参数的日志收益最大。
// 全部日志宏均支持 `--log-only=<模块名>` 进行运行期过滤。
//
// | 开关 | 状态 | 输出内容 | 频率 |
// |---|---|---|---|
// | `HZMIR_LOG_YOLO` | ✅ | YOLO 逐帧细节：推理耗时、两道阈值、完整漏斗、峰值位置 | 每帧 2 行 |
// | `HZMIR_LOG_DETECTOR` | ✅ | 传统检测器灯条与装甲板配对细节 | 每帧多行 |
// | `HZMIR_LOG_EKF` | ✅ | EKF 新息（NIS）、收敛与发散判定 | 高 |
// | `HZMIR_LOG_TRACKER` | ✅ | 跟踪状态机切换（lost / detecting / tracking / temp_lost） | 低 |
// | `HZMIR_LOG_TARGET` | ✅ | 目标选择、跳变、小陀螺判据、发散原因、首次收敛 | 低 |
// | `HZMIR_LOG_PLANNER` | ✅ | MPC 迭代次数、重合度、加速度上限、弹道 | 中 |
// | `HZMIR_LOG_AIMER` | ✅ | 瞄点选择、弹道无解 | 中 |
// | `HZMIR_LOG_SHOOTER` | ✅ | 开火判据未通过的原因 | 中 |
// | `HZMIR_LOG_BUFF` | ✅ | 打符检测、拟合、预测 | 中 |
// | `HZMIR_LOG_TI` | ✅ | 时序积分器 `TemporalIntegrator` | 低 |
// | `HZMIR_LOG_TGD` | ✅ | 传统检测的目标引导检测（TGD） | 中 |
// | `HZMIR_LOG_CAMERA` | ✅ | 相机队列满导致的丢帧（真实丢帧来源） | 低 |
// | `HZMIR_EXP_NO_TRAD_SAVE` | ✅ | 关闭传统检测器的落图（见 §四） | — |
//
// 已删除的开关：`HZMIR_LOG_BOARD`、`HZMIR_LOG_IMU`、`HZMIR_LOG_SINK`。
// 三者对应模块均已有等效日志，重复定义会造成误导，改用运行期过滤：
//   · 下位机收发 —— `io/board/gimbal/gimbal.cpp` 已有日志，用 `--log-only=gimbal`
//   · sink 生命周期 —— `ImageSink` / `WindowSink` 已有 info 级日志，用 `--log-only=sink`
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// §二  按场景选择开关
//
// 场景 A —— 识别率不足，或检测不到装甲板
//   1. 优先使用运行期概览日志，无需重新编译：
//        tools/scripts/run.sh infantry --log-only=yolo --video=<录像路径>
//      输出两类信息，均已限频：
//        "本帧无候选：objectness 峰值 X < 阈值 Y"（每 60 帧一条）
//        "objectness 通过 N → 输出 M（滤掉 not_armor/conf/type）"（每 30 帧一条）
//   2. 若信息不足，开启 `HZMIR_LOG_YOLO`（每帧 2 行，信息最完整）
//   3. 传统检测器（`detector_impl=traditional`）开启 `HZMIR_LOG_DETECTOR`
//   4. "检测到蓝色装甲板但跟踪器不接受"属于敌我颜色配置问题，与日志无关：
//        tools/scripts/run.sh infantry --enemy-color=blue ...
//      敌我颜色只影响跟踪阶段的过滤，不影响检测。详见 docs/19。
//
// 场景 B —— EKF 状态异常、目标跳变、小陀螺打不准
//   1. 开启 `HZMIR_LOG_TRACKER`（频率低），确认状态机是否进入 tracking
//   2. 开启 `HZMIR_LOG_EKF`（输出最频繁，建议配合 `--log-only=ekf`）
//   3. 目标选择问题开启 `HZMIR_LOG_TARGET`（含发散原因与首次收敛）
//
// 场景 C —— 可跟踪但打不中，或不开火
//   1. 开启 `HZMIR_LOG_SHOOTER`，输出开火判据的完整依据
//      判据为 `traj_err < fire_thresh`，默认阈值 0.003，见 `planner/mpc.cpp`
//   2. 开启 `HZMIR_LOG_PLANNER`（迭代次数、重合度、加速度上限）
//   3. 开启 `HZMIR_LOG_AIMER`（瞄点选择、弹道无解）
//   4. 云台不动时无需开启宏，检查是否出现以下 info 日志：
//        [Board] first control command: fire=... yaw=...
//      长时间缺失说明未执行到发送环节（该日志一处覆盖四个兵种）
//
// 场景 D —— 打符：开启 `HZMIR_LOG_BUFF`
//
// 场景 E —— 硬件层
//   下位机：使用 `--log-only=gimbal` 过滤现成日志，
//     `io/board/gimbal/gimbal.cpp` 已覆盖串口收发、CRC 校验、线程启停。
//   相机丢帧：开启 `HZMIR_LOG_CAMERA`，报告采集队列满导致的丢帧。
//     若持续出现该日志，说明消费者（自瞄主循环）跟不上采集速率。
//
// 场景 F —— 确认 sink 挂载情况：使用 `--log-only=sink`
//   `[ImageSink]` 与 `[WindowSink]` 的生命周期已有 info 级日志。
//
// 场景 G —— 批量调参时节省磁盘：开启 `HZMIR_EXP_NO_TRAD_SAVE`
// ═══════════════════════════════════════════════════════════════════════════

// ═══════════════════════════════════════════════════════════════════════════
// §三  日志开关
//
// `#define` 即开启，注释掉即关闭。目前共 12 个，默认全部关闭。
// 展开为 `LOG_xxx(...)` 的位置在 `utils/log/debug_config.hpp`。
// 建议：每帧刷屏的细节保持关闭，低频的状态变化按需开启。
//
// 每个开关下方给出该日志的【实际输出格式】与【各字段含义】，
// 便于开启后直接对照阅读，无需另行查代码。
// ═══════════════════════════════════════════════════════════════════════════

// ── 检测链路 ──

// #define HZMIR_LOG_YOLO
// 输出两行（每帧）。
//
// 第 1 行：① 推理 4220us ｜ ② 门槛 obj 0.70 → conf 0.80（NMS 0.30） ｜ ④ 峰值 0.842 @归一化(0.595,0.642) anchor#21583
//   ① 推理        `infer()` 单次推理耗时。注意与 CSV 的 `det_t_infer_us` 不同，后者为整段检测
//   ② 门槛        objectness 阈值 0.70 → 分类置信度阈值 0.80（NMS IoU 阈值 0.30）
//                 概览日志仅报告第一道阈值，不说明第二道容易造成误判
//   ④ 峰值        本帧最高 objectness、其归一化图像坐标与所在 anchor 序号
//                 用于区分图像质量问题（峰值居中）与背景误检（峰值在边缘）
//
// 第 2 行：③ 漏斗 25200 anchor → >0.5:7 >0.3:13 >0.1:19 → pass 1 → NMS 1 → 输出 1（丢 not_armor 0 / conf 0 / type 0）
//   25200 anchor 候选总数。输入 640x640，三个检测头 80²+40²+20² 格 x 3 anchor
//   >0.5 / >0.3 / >0.1   objectness 超过该值的 anchor 数，反映"离能用还有多远"
//   pass          通过 objectness 阈值 0.70 的候选数
//   NMS           非极大值抑制后的存活数（合并重叠框）
//   输出          最终装甲板数
//   丢 not_armor  被判为"不是装甲板"而丢弃的数量
//   丢 conf       分类置信度不足（<= 0.80）而丢弃的数量
//   丢 type       大/小装甲板类型不符（`check_type`）而丢弃的数量
//
// 诊断示例：`pass 5 → 输出 0（丢 conf 5）` 表示候选全被分类置信度拦下，
// 即《无数字装甲板为何识别不到》所描述的情形。

// #define HZMIR_LOG_DETECTOR
// 传统检测器（`detector_impl=traditional`）的灯条与装甲板配对细节。
// 输出样例：`See pattern 5` —— 识别到编号 5 的图案时提示。
// 说明：目前调用点较少；传统检测器的主要信息在概览日志中。

// ── 跟踪与估计 ──

// #define HZMIR_LOG_EKF
// EKF 层信息，输出最频繁。
// 输出样例：`[Target] r=0.041, l=0.000`
//   r   装甲板半径估计（`ekf_.x[8]`），合法区间 (0.05, 0.5)
//   l   另一侧板宽估计（`ekf_.x[9]`），合法条件 r+l ∈ (0.05, 0.5)
// 两者越界即判定为"发散"，见 `LOG_TARGET` 的输出。
// 其他输出：`[Target] Bad Converge Found!` —— 收敛检查未通过。

// #define HZMIR_LOG_TRACKER
// 跟踪状态机事件，频率低。输出样例：
//   `auto_aim switch target to three`   切换目标为编号 3
//   `[Tracker] Target diverged!`        目标发散，状态退回 lost
//   `omniperception find higher priority target`  发现更高优先级目标
//   `[Tracker] 未配置 priority_mode（同济行为）`  启动期提示
// 状态机取值：lost / detecting / tracking / temp_lost（见 CSV 的 `trk_state`）

// #define HZMIR_LOG_TARGET
// 目标级事件，频率低。输出样例：
//   `✅ 首次收敛：update_count=4 r=0.180 l=0.000 yaw=132.6° vyaw=-0.70 name=5`
//     update_count  已关联的观测帧数
//     r / l         半径与板宽估计（同 `LOG_EKF`）
//     yaw           目标朝向（度）；vyaw 为角速度（弧度/秒）
//     name          装甲板编号（`ArmorName` 枚举值）
//   `发散：r=0.041 l=0.000（r+l=0.041）合法区间 r,l+r ∈ (0.05, 0.5) name=5`
//     给出越界的具体数值，用于区分"半径估计飞出"与"板宽估计飞出"

// ── 规划与射击 ──

// #define HZMIR_LOG_PLANNER
// MPC 解算结果。输出样例：
//   `trajectory_impl = ideal (ideal)`   当前弹道实现（启动期提示）
//   `yaw_iters=1 pitch_iters=10 重合度=1.000 acc_max=0.00 飞行=0.0426s 距离=0.94m`
//     yaw_iters / pitch_iters  MPC 求解器迭代次数（接近上限说明未收敛）
//     重合度                  前瞻段重合度，0~1，越高越接近参考轨迹
//     acc_max                 规划轨迹最大加速度，用于检查是否超物理上限
//     飞行                    弹丸飞行时间 `t_fly`
//     距离                    最近装甲板水平距离

// #define HZMIR_LOG_AIMER
// 瞄点选择。仅在弹道无解时输出，样例：
//   `[Aimer] unsolvable traj0: speed=22.00 d=1.23 z=0.45`
//     traj0 / iter  第 0 次（初始解）或第 N 次迭代
//     speed         弹速（m/s）；d 水平距离（m）；z 高度（m）
//   判据为 `Trajectory::unsolvable`，无解时本轮不下发瞄点。

// #define HZMIR_LOG_SHOOTER
// 开火判据。输出样例：
//   `fire=true traj_err=0.00000 / thresh=0.00300 弹速=22.00 飞行=0.0426s 距离=0.94m`
//     fire      本帧是否满足开火条件
//     traj_err  前瞻点的「参考轨迹 vs 规划轨迹」误差
//     thresh    开火阈值（`fire_thresh`，默认 0.003）
//     弹速 / 飞行 / 距离  同 `LOG_PLANNER`
//   判据为 `traj_err < fire_thresh`，等价于"云台能跟上目标"。
//   `fire=false` 时可直接读出差距，判断是"差一点"还是"完全跟不上"。

// ── 打符 ──

// #define HZMIR_LOG_BUFF
// 打符模块。输出样例：
//   `[Aimer] Unsolvable trajectory0: 22.00 1.23 0.45`  弹道无解（弹速 水平距离 高度）
//   `[Target] 丢失buff`                                 目标丢失
//   `[Target] 小符角度发散spd: 12.34`                    角度发散，值为角速度（度/秒）

// ── 硬件 ──

// #define HZMIR_LOG_CAMERA
// 相机采集队列满导致的丢帧。输出样例：
//   `HikRobot 队列满 ⇒ 丢帧 ×120（消费者跟不上采集；这是丢帧的真实来源）`
//     驱动名  hikrobot / mindvision / usbcamera
//     ×N      累计丢帧次数（每 60 次输出一条）
//   原理：采集队列 `queue_(1)` 只缓冲 1 帧，模板默认 `PopWhenFull = false`，
//   队列满时丢弃**新**帧。接入本回调前丢帧完全静默，只能观察到帧率下降而无从判断原因。
//   本开关只做上报，不改变队列容量与丢弃策略。
//   若持续出现，说明消费者（自瞄主循环）跟不上采集速率。

// ── 轮子 utils/wheels/ ──

// #define HZMIR_LOG_TI
// 时序积分器 `TemporalIntegrator`。输出样例：
//   `[TI] Frame 123: 高速 (v=15.3 m/s) → 旁路 TI`      目标过快，跳过时序积分
//   `[TI] Frame 123: window=5 verified=3/5`             窗口内 5 条历史，验证通过 3 条
//     window    历史窗口长度；verified 通过验证数；末项为窗口内总条目数

// #define HZMIR_LOG_TGD
// 传统检测的目标引导检测（`tgd.cpp`）。输出样例：
//   `[TGD] Adaptive threshold: 12.34 (mean=8.10, std=3.20)`
//     自适应二值化阈值，及其依据的局部均值与标准差
//   `[TGD] Frame 123: centers=4, motion_intensity=2.35%`
//     centers           检出的运动中心数
//     motion_intensity  运动区域占画面的百分比

// ═══════════════════════════════════════════════════════════════════════════
// §四  算法实验开关
//
// 新增一个实验的步骤：
//   1. 实现置于 `test/function/exp_<名称>.{hpp,cpp}`
//   2. 在下方添加 `#define HZMIR_EXP_<名称>`
//   3. 在对应文件的**装配点**添加 `#ifdef` 入口，不得写入算法内部
//
// CMake 会自动解析本文件：检测到某个 `HZMIR_EXP_*` 被定义时，自动在全项目定义
// 该宏，并将对应的 `exp_*.cpp` 纳入构建，无需手工修改 CMakeLists。
//
// 生命周期：实验验证完成后必须二选一。
//   · 转正 —— 实现移入 `core/<角色>/`，删除开关与 `#ifdef` 入口
//   · 放弃 —— 删除 `exp_*.cpp`、开关与 `#ifdef` 入口
// 实验开关不得长期保留，否则将从"实验"演变为无人敢删的隐藏功能。
//
// 关闭开关时无残留：`exp_*.cpp` 不参与编译，`#ifdef` 内的代码不存在，
// 既无编入而未执行的死代码，也无运行期分支。
//
// 完整模板（含 `init()` 与装配点写法）见 docs/16。
// ═══════════════════════════════════════════════════════════════════════════

// ── 实验：关闭传统检测器的落图 ──
// 背景：`Detector::save()` 目前无条件落图（`detector.cpp` 的 check_name / check_type），
// 分类置信度不足或类型异常时保存一张 JPEG，用于分类器迭代。三个 YOLO 版本的
// 同类调用处于注释状态，两者行为不一致。
// 默认保持同济行为，即执行落图；开启本开关后关闭落图，适用于批量调参以节省磁盘与时间。
// #define HZMIR_EXP_NO_TRAD_SAVE    // 关闭传统检测器的落图

// ═══════════════════════════════════════════════════════════════════════════
// §五  兼容开关
//
// 与 §四 的区别：§四 为临时实验，本段为长期的兼容与回退路径，可以长期存在。
//
// 现状：本项目在源码中硬编码的偏差，统一由运行期参数 `--tongji` 控制，
// 见 `utils/config/tongji_flags.hpp`，因此本段目前为空。
// 若出现只能在编译期决定的兼容点，在此处添加。
// ═══════════════════════════════════════════════════════════════════════════

#endif  // HZMIR_CORE_DEBUG_HPP
