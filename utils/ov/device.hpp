/**
 * @file utils/ov/device.hpp
 * @brief ⭐ 轮子：OpenVINO 设备解析（**请求的设备不可用时回退 CPU**）
 *
 * ## 解决的问题（同济源码的健壮性缺口）
 * 同济的 `yolos/*.cpp` 直接：
 * ```cpp
 * device_ = yaml["device"].as<std::string>();           // yaml 写 "GPU"
 * compiled_model_ = core_.compile_model(model, device_, ...);
 * ```
 * 在**无 GPU 的机器上** `compile_model` 会抛 `ov::Exception` → 进程 `abort`：
 * ```
 * [GPU] Can't get PERFORMANCE_HINT property as no supported devices found
 * ```
 *
 * ⭐ 本函数把「请求的设备」解析成「**实际可用的**设备」：
 *   · 请求的设备在 `get_available_devices()` 里 → 原样返回
 *   · 不在 → **warn + 回退 `CPU`**（保持程序可跑）
 *   · 请求 `"AUTO"` → 原样返回（交给 OpenVINO 自己调度）
 *   · yaml 里没写 / 写空 → 返回 `"CPU"`（与同济 `yolo11_buff` 的硬编码一致）
 *
 * ⚠️ **归类为「纯修复」**：只在同济代码**本来会崩**的路径上改变行为。
 *    想恢复同济的严格行为：`tools::set_strict_device(true)` → 不可用就抛异常。
 */
#ifndef HZMIR_UTILS_OV_DEVICE_HPP
#define HZMIR_UTILS_OV_DEVICE_HPP

#include <string>
#include <vector>

#include <openvino/openvino.hpp>

namespace tools
{

/// @brief 已探测到的可用设备（`get_available_devices()`，含 "CPU"）
std::vector<std::string> available_devices(ov::Core & core);

/// @brief 解析设备名（不可用则回退 CPU 并警告）
/// @param requested yaml 的 `device`（如 "GPU" / "CPU" / "AUTO" / 空）
std::string resolve_device(ov::Core & core, const std::string & requested);

/// @brief 描述可用设备（启动日志用，形如 `CPU;GPU.0`）
std::string describe_devices(ov::Core & core);

/// ⭐ 严格模式：请求的设备不可用就抛异常（= 同济原行为）。默认 `false`（回退 CPU）
void set_strict_device(bool on);
bool strict_device();

}  // namespace tools

#endif  // HZMIR_UTILS_OV_DEVICE_HPP
