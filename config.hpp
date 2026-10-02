/**
 * @file config.hpp
 * @brief 编译期配置的**唯一入口**（宏规范）
 *
 * 规则：
 *  ① 所有宏只在这里定义，其他文件只 `#include "config.hpp"`
 *  ② 允许出现 `#ifdef` 的只有 4 处：
 *       config.hpp / 装配函数 / CMakeLists.txt / 测试入口
 *     ⚠️ 算法 .cpp 里出现 #ifdef 视为违规
 *  ③ 宏只干 3 类事：决定「编不编」/「默认值」/ 互斥检查
 *     "同接口换实现" 一律走运行期参数（yaml 的 *_impl / CLI --robot）
 */
#ifndef HZMIR_CONFIG_HPP
#define HZMIR_CONFIG_HPP

// ═══════════════════════════════════════════════════════════════
// A. 平台 / 架构   —— 决定链接哪个 .so
// ═══════════════════════════════════════════════════════════════
#if !defined(PLATFORM_X86_64) && !defined(PLATFORM_ARM64)
#  define PLATFORM_X86_64          // 默认 x86_64；交叉编译时改这里
#endif

// ═══════════════════════════════════════════════════════════════
// B. 依赖 / 后端
// ═══════════════════════════════════════════════════════════════
#define HAS_OPENVINO                 // YOLO 推理（本机已装 2024.6）
#define HAS_ROS2                    // ⭐ W35：哨兵导航桥（上行目标位置 + 下行无敌/集火）
// #define HAS_DM_IMU
// #define HAS_SOCKETCAN

// 相机后端（至少一个）—— 本机无相机，先只开 webcam
// #define HAS_HIKROBOT
// #define HAS_MINDVISION
// #define HAS_WEBCAM   // ⚠️ 同济无 webcam 驱动
#define HAS_USBCAMERA

// ═══════════════════════════════════════════════════════════════
// C. 功能组
// ═══════════════════════════════════════════════════════════════
#define ENABLE_AUTO_AIM
#define ENABLE_AUTO_BUFF
// #define ENABLE_OMNIPERCEPTION      // ⚠️ 4 相机专用，单相机兵种不需要

// ═══════════════════════════════════════════════════════════════
// D. 调试
// ═══════════════════════════════════════════════════════════════
// ⭐⭐⭐ W60：**这个宏已废弃（保留仅为兼容旧构建脚本）**
//   原因：L3（存图/可视化窗口）现在**总是编入**，纯运行期控制（按键 1/4、--debug-img/--debug-window）。
//   ⚠️ 原来的问题是：Release 下 L3 被宏关掉 → **按键 1/4 永远按不动、还得重编**，
//      而帮助文本**照样列着它们** → 误导用户（实测反馈："不要搞这个误导人了"）。
//   ⚠️ 而 `libopencv_highgui` **本来就已经链接**（`ldd` 可查）→ 这个宏**没有省依赖/省体积的收益**。
//   ⇒ 唯一保留的运行期差别：`WindowSink` 在**无 DISPLAY** 时不可用（那是运行期检查）。
// #define DEBUG_L3_ENABLE            // ← 已废弃，定义与否都不影响 L3 是否编入
#define DEBUG_SINK_CSV
#define DEBUG_SINK_PLOTJUGGLER
// #define DEBUG_SINK_OPENCV
// #define DEBUG_SINK_ROS             // ⚠️ 需要 HAS_ROS2

// ═══════════════════════════════════════════════════════════════
// E. 运行期默认值
// ═══════════════════════════════════════════════════════════════
#define CONFIG_DEFAULT_YAML  "params/infantry.yaml"
#define CONFIG_DEFAULT_ROBOT "infantry"

// ═══════════════════════════════════════════════════════════════
// F. 互斥检查（把运行期 bug 变成编译期错误）
// ═══════════════════════════════════════════════════════════════
#if defined(PLATFORM_X86_64) && defined(PLATFORM_ARM64)
#  error "只能选一个平台"
#endif

#if !defined(HAS_HIKROBOT) && !defined(HAS_MINDVISION) && \
    !defined(HAS_USBCAMERA) && !defined(HAS_WEBCAM)
#  error "至少要有一个相机后端"
#endif

#if defined(DEBUG_SINK_ROS) && !defined(HAS_ROS2)
#  error "DEBUG_SINK_ROS 需要 HAS_ROS2"
#endif

// ⭐⭐ W35：ROS2 的「CMake ↔ config.hpp」一致性检查
//   修复前 `io/CMakeLists.txt` 用 `if(HAS_ROS2)` 判断，而 CMake 里没有这个变量
//   → io/ros2/ **从未编译**，但 config.hpp 的宏却"看起来"是开的（静默失效）。
//   现在两侧必须一致，不一致就**编译期报错**。
#if defined(HZMIR_CMAKE_HAS_ROS2) && !defined(HAS_ROS2)
#  error "CMake 的 HZMIR_WITH_ROS2=ON，但 config.hpp 未定义 HAS_ROS2 → 请打开 config.hpp 的 HAS_ROS2"
#endif
#if !defined(HZMIR_CMAKE_HAS_ROS2) && defined(HAS_ROS2)
#  error "config.hpp 定义了 HAS_ROS2，但 CMake 未开 → 请用 -DHZMIR_WITH_ROS2=ON 重新配置"
#endif

#if defined(ENABLE_OMNIPERCEPTION) && !defined(HAS_USBCAMERA)
#  error "ENABLE_OMNIPERCEPTION 需要 HAS_USBCAMERA（4 相机）"
#endif

#if !defined(ENABLE_AUTO_AIM) && !defined(ENABLE_AUTO_BUFF)
#  error "至少要启用一个功能组"
#endif

// ═══════════════════════════════════════════════════════════════
// G. 派生宏（自动推导）
// ═══════════════════════════════════════════════════════════════
#if !defined(NDEBUG)
#  ifndef DEBUG_L3_ENABLE
#    define DEBUG_L3_ENABLE
#  endif
#  ifndef DEBUG_SINK_OPENCV
#    define DEBUG_SINK_OPENCV
#  endif
#endif

#endif  // HZMIR_CONFIG_HPP
