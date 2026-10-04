/**
 * @file utils/log/debug_config.hpp
 * @brief **开关【展开器】** —— 把 `core/debug.hpp` 的开关翻译成 `LOG_xxx(...)` 宏（W104）
 *
 * ## ⚠️ 开关**不在本文件**了
 * ```
 * core/debug.hpp              ← ⭐⭐【开关与实验总控】所有 #define 都在那
 * utils/log/debug_config.hpp  ← ⭐【你在这里】只负责【展开】（需要 logger()）
 * core/debug_node.hpp         ← 【数据面】FrameDebug（零宏）
 * ```
 * ⭐ 为什么拆开：`core/debug.hpp` 要**零依赖**（让 `utils/` 也能 include 它拿开关）。
 *   而展开需要 `tools::logger()` ⇒ 展开只能留在 `utils/log/`。
 *
 * ## 为什么需要（替代"运行期热键切模块"）
 * W95 做了运行期按模块过滤（`--log-off` / 热键 `n`）—— **灵活，但有两个问题**：
 *   ① ⚠️ **热键记不住**（用户实测反馈："这个我不想热插拔很难记忆"）
 *   ② ⚠️ **过滤发生在格式化【之后】** ⇒ 参数**照样求值**（实测 `49 ns/次` vs 零成本）
 *
 * ⇒ 本文件提供**编译期开关**：关掉的模块，**日志语句连同参数一起消失**。
 *
 * ## 设计（参考 cyberdog_race 的 `debug_config.hpp`）
 * ```
 * ① 集中开关：一行一个模块，`#define` = 开，注释掉 = 关
 * ② 每个模块一对宏：`LOG_XXX(...)`（关时展开成【空】）
 * ③ 使用处直接写 `LOG_YOLO("...", a, b);` —— ⭐ **不需要 `#ifdef`**
 * ```
 *
 * ## ⭐ 与「宏纪律」的关系（不冲突）
 * 本项目的宏规范说「宏只干 3 类事：编不编 / 默认值 / 互斥检查」，且
 * `#ifdef` 只允许出现在 4 处 —— ⚠️ **那是针对【业务逻辑】的**。
 * 日志开关属于「编不编」这一类，且**开关集中在本文件**，
 * 使用处写的是**普通函数调用样式的宏**（无 `#ifdef`），
 * ⇒ 既守住了纪律，又拿到了零成本。
 *
 * ## ⚠️ 为什么 W60 废弃了 `DEBUG_L3_ENABLE`，这里却要做宏开关？
 * | | L3（存图/窗口） | 本文件（细节日志） |
 * |---|---|---|
 * | 关掉的后果 | ⚠️ **按键按不动**（但帮助文本还列着）→ **误导** | 只是少打日志，**无语义变化** |
 * | 省下的开销 | **0**（依赖本来就链接） | ⭐ **真实**（参数求值 + 格式化） |
 * | 会不会让人误判 | ⚠️ **会**（以为功能坏了） | ✅ **不会** |
 * ⇒ 判据是：**关掉后是否会让用户误判"功能坏了"**。日志不会，功能开关会。
 *
 * ## ⭐ 反查：别人一般会去哪里找你（W103 补）
 * ```
 *   core/debug.hpp            ← 【数据面】FrameDebug 聚合（⭐ 零宏，不是开关）
 *   utils/log/debug_config.hpp ← ⭐⭐ **你在这里** —— 日志开关总控
 *   utils/log/log_filter.hpp   ← 【运行期】--log-off / --log-only
 *   utils/debug/debug_sink.hpp ← 【数据去向】SinkHub
 * ```
 * ⭐ 快速自查：跑 `--print-config`，第 ④ 节会列出**当前哪些 `LOG_*` 是开的**。
 *
 * ## 怎么加一个新模块
 * 1. 下面「开关」段加一行 `#define HZMIR_LOG_你的模块`
 * 2. 在对应的 `#ifdef` 段加一对宏：
 *    ```cpp
 *    #ifdef HZMIR_LOG_你的模块
 *      #define LOG_你的模块(...) tools::logger()->debug("[tag] " __VA_ARGS__)
 *    #else
 *      #define LOG_你的模块(...)
 *    #endif
 *    ```
 * 3. 把源码里的 `tools::logger()->debug("[tag] ...", ...)` 换成 `LOG_你的模块("...", ...)`
 */
#ifndef HZMIR_UTILS_LOG_DEBUG_CONFIG_HPP
#define HZMIR_UTILS_LOG_DEBUG_CONFIG_HPP

#include "utils/log/logger.hpp"

// ⭐⭐⭐ W104：**开关已经搬到 `core/debug.hpp`（开关与实验总控）**
//
// 本文件现在**只干一件事**：把那些开关**展开**成 `LOG_xxx(...)` 宏。
// ⚠️ 为什么展开不能也搬到 `core/debug.hpp`：展开需要 `tools::logger()`，
//   而 `core/debug.hpp` 是**零依赖**的（谁都能 include）⇒ 展开留在这里。
//
// ⭐ 想开某个模块的日志 ⇒ 改 **`core/debug.hpp` 的 §一**（不是本文件）
#include "core/debug.hpp"

// ═══════════════════════════════════════════════════════════════
// 宏定义（⚠️ 下面的格式统一：关时展开成【空】，连参数都不求值）
// ═══════════════════════════════════════════════════════════════

#ifdef HZMIR_LOG_YOLO
#  define LOG_YOLO(...) tools::logger()->debug("[yolo] " __VA_ARGS__)
#else
#  define LOG_YOLO(...)   // ⭐ 展开为空 ⇒ 零成本
#endif

#ifdef HZMIR_LOG_DETECTOR
#  define LOG_DETECTOR(...) tools::logger()->debug("[detector] " __VA_ARGS__)
#else
#  define LOG_DETECTOR(...)
#endif

#ifdef HZMIR_LOG_EKF
#  define LOG_EKF(...) tools::logger()->debug("[ekf] " __VA_ARGS__)
#else
#  define LOG_EKF(...)
#endif

#ifdef HZMIR_LOG_TRACKER
#  define LOG_TRACKER(...) tools::logger()->debug("[tracker] " __VA_ARGS__)
#else
#  define LOG_TRACKER(...)
#endif

#ifdef HZMIR_LOG_TARGET
#  define LOG_TARGET(...) tools::logger()->debug("[target] " __VA_ARGS__)
#else
#  define LOG_TARGET(...)
#endif

#ifdef HZMIR_LOG_PLANNER
#  define LOG_PLANNER(...) tools::logger()->debug("[planner] " __VA_ARGS__)
#else
#  define LOG_PLANNER(...)
#endif

#ifdef HZMIR_LOG_AIMER
#  define LOG_AIMER(...) tools::logger()->debug("[aimer] " __VA_ARGS__)
#else
#  define LOG_AIMER(...)
#endif

#ifdef HZMIR_LOG_SHOOTER
#  define LOG_SHOOTER(...) tools::logger()->debug("[shooter] " __VA_ARGS__)
#else
#  define LOG_SHOOTER(...)
#endif

#ifdef HZMIR_LOG_BUFF
#  define LOG_BUFF(...) tools::logger()->debug("[buff] " __VA_ARGS__)
#else
#  define LOG_BUFF(...)
#endif

#ifdef HZMIR_LOG_CAMERA
#  define LOG_CAMERA(...) tools::logger()->debug("[camera] " __VA_ARGS__)
#else
#  define LOG_CAMERA(...)
#endif



#ifdef HZMIR_LOG_TI
#  define LOG_TI(...) tools::logger()->debug("[TI] " __VA_ARGS__)
#else
#  define LOG_TI(...)
#endif

#ifdef HZMIR_LOG_TGD
#  define LOG_TGD(...) tools::logger()->debug("[TGD] " __VA_ARGS__)
#else
#  define LOG_TGD(...)
#endif


#endif  // HZMIR_UTILS_LOG_DEBUG_CONFIG_HPP
