/**
 * @file core/auto_aim/target/target_plugins_factory.hpp
 * @brief ⭐⭐ W42：从 **yaml** 装配 `Target` 的 3 个策略槽位
 *
 * ## 为什么需要它（补上一个真实缺口）
 * W23 把 3 个槽位迁进来了，但**只有 C++ API 能设**（`set_*_estimator(...)`），
 * **yaml 里一个键都没有** → 想 A/B 还是得改代码重编译 →
 * **插件设计的那个目的（可复现的 A/B 实验）实际没达成**。
 *
 * 对比：**弹道槽位是接好了的**（`trajectory_impl` 写 yaml 就行），
 * 所以弹道能做量化实验（W30 就是靠它量出「步长 1e-4→1e-3 精度 0 差异、快 10×」）。
 *
 * ## yaml 长这样（**不写 = 同济行为**）
 * ```yaml
 * target_plugins:
 *   angular_velocity: ekf_state   # ekf_state(同济) | ekf | visual_diff | imu_fusion
 *   process_noise:    fixed       # fixed(同济)     | nis
 *   measurement:      passthrough # passthrough(同济) | median
 *   # 可选参数
 *   imu_alpha: 0.7                # ImuFusion 的短期权重
 *   med_window: 3                 # MedianFilter 的窗口
 * ```
 *
 * ⭐ **默认装配 = `ekf_state` + `fixed` + `passthrough` = 完全同济行为**
 */
#ifndef HZMIR_CORE_AUTO_AIM_TARGET_PLUGINS_FACTORY_HPP
#define HZMIR_CORE_AUTO_AIM_TARGET_PLUGINS_FACTORY_HPP

#include <string>

#include <yaml-cpp/yaml.h>

#include "core/auto_aim/target/target.hpp"   // ⭐ 需要 Target 的完整定义
#include "core/auto_aim/target/target_plugins.hpp"

namespace auto_aim
{

/// @brief 按 yaml 装配 3 个槽位（缺省 = 同济行为）；会打日志说明选了哪个
void apply_target_plugins_from_yaml(Target & target, const YAML::Node & yaml);

/// @brief 只解析名字（便于测试/日志）
std::string parse_plugin_name(
  const YAML::Node & yaml, const char * key, const char * fallback);

}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_TARGET_PLUGINS_FACTORY_HPP
