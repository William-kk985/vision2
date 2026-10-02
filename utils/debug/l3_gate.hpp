/**
 * @file utils/debug/l3_gate.hpp
 * @brief ⭐⭐⭐ **全局 L3 图像门控** —— 供「拿不到 `SinkHub` 引用」的深层代码使用
 *
 * ## 解决什么问题（W61）
 * `core/auto_aim/detector/` 与 `core/auto_aim/detector/yolos/` 里有 **5 处**
 * **无条件、每帧执行**的 `cv::imshow`（**同济调试残留**，他们自己注释掉了 7 处、漏了这 5 处）：
 * ```cpp
 * cv::imshow("binary_img", binary_img);   // detector.cpp:42
 * cv::imshow("detection",  detection);    // detector.cpp:383
 * cv::imshow("detection",  detection);    // yolo11/yolov5/yolov8.cpp
 * ```
 * ⚠️ **它们完全绕过 `SinkHub`** → 按键 `1` 关不掉它们、`--debug-window` 也管不到它们。
 *
 * ## 实测代价（本机，640×360）
 * | 操作 | 有 DISPLAY | **无 DISPLAY（赛场无头）** |
 * |---|---:|---:|
 * | `cv::resize`（附带） | 0.133 ms | **0.133 ms** |
 * | `cv::imshow` | ⚠️ **0.703 ms** | 0.016 ms |
 * | `+ waitKey(1)` | ⚠️ **1.335 ms** | 0.010 ms |
 * ⇒ **有显示器时 ≈1.4 ms/帧 = 14% 帧预算**；无头时也白花 0.133 ms 的 `resize`。
 *
 * ## 机制
 * `SinkHub` 是「谁要图」的唯一真相源 → 它的 `add`/`remove` **同步**本门控；
 * 深层代码只需问一句：
 * ```cpp
 * if (tools::l3_image_wanted()) cv::imshow("detection", detection);
 * ```
 * ⭐ 没人要图（没按 `1`、没传 `--debug-window`）→ **连 `resize` 都不做**。
 *
 * ## 为什么用全局而不是传参
 * detector / yolo 是**深层算法代码**，为了一个调试开关把 `SinkHub&` 一路传下去会污染所有签名。
 * ⚠️ 这与「`utils/` 不碰业务类型」不冲突 —— 本头文件**只依赖 `<atomic>`**，无业务类型。
 */
#ifndef HZMIR_UTILS_DEBUG_L3_GATE_HPP
#define HZMIR_UTILS_DEBUG_L3_GATE_HPP

#include <atomic>

namespace tools
{

namespace detail
{
/// ⭐ 全局门控标志（release-acquire 语义足够：写一次、多线程读）
inline std::atomic<bool> & l3_image_wanted_flag()
{
  static std::atomic<bool> flag{false};
  return flag;
}
}  // namespace detail

/// @brief 设置门控（由 `SinkHub::add`/`remove` 调用；一般不用手动调）
inline void set_l3_image_wanted(bool on) noexcept
{
  detail::l3_image_wanted_flag().store(on, std::memory_order_release);
}

/// @brief ⭐ **有没有人要图像** —— 深层代码（detector/yolo）在 `imshow`/`resize` 前问它
inline bool l3_image_wanted() noexcept
{
  return detail::l3_image_wanted_flag().load(std::memory_order_acquire);
}

}  // namespace tools

#endif  // HZMIR_UTILS_DEBUG_L3_GATE_HPP
