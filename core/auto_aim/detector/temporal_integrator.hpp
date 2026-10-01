/**
 * @file core/auto_aim/detector/temporal_integrator.hpp
 * @brief 把泛型轮子 `tools::TemporalIntegrator` 绑到 `auto_aim::Armor` 上
 *
 * ⭐ **依赖方向**：`utils/wheels/track/` 只认识模板参数 `T`；
 *    4 处 `Armor` 字段访问全部收敛到本文件的 **`TemporalTraits<Armor>` 特化**里
 *    —— 这是「轮子 + 适配」模式在**类型层面**的体现。
 *
 * 原实现把这 4 处**硬编码**在 `temporal_integrator.cpp` 里（分别散在 3 个函数中），
 * 所以那个类**没法复用**、也没法单独测试。
 */
#ifndef HZMIR_CORE_AUTO_AIM_DETECTOR_TEMPORAL_INTEGRATOR_HPP
#define HZMIR_CORE_AUTO_AIM_DETECTOR_TEMPORAL_INTEGRATOR_HPP

#include "core/types.hpp"
#include "utils/wheels/track/temporal_integrator.hpp"

namespace tools
{

/// ⭐ `Armor` 的时序积分策略（4 处字段访问全部集中在这里）
template <>
struct TemporalTraits<auto_aim::Armor>
{
  /// 空间距离：用世界系 3D 欧氏距离（原 `computeSpatialDistance`）
  static double distance(const auto_aim::Armor & a, const auto_aim::Armor & b)
  {
    const Eigen::Vector3d d = a.xyz_in_world - b.xyz_in_world;
    return d.norm();
  }

  /// 是否同一类目标：type + color + name
  /// ⚠️ 原实现注释强调过：**必须查 name** —— 大/小装甲板 type 可能都是 small，
  ///    只查 type/color 会把 enemy_one 和 enemy_three 混在一起，引起跳变
  static bool same_object(const auto_aim::Armor & a, const auto_aim::Armor & b)
  {
    return a.type == b.type && a.color == b.color && a.name == b.name;
  }

  /// 跨帧稳定 ID（原 `generateArmorID`）
  static int id(const auto_aim::Armor & a)
  {
    return static_cast<int>(a.type) * 1000 + static_cast<int>(a.color) * 100 +
           static_cast<int>(a.name);
  }

  /// 图像坐标（用于记轨迹）
  static cv::Point2f image_center(const auto_aim::Armor & a) { return a.center; }

  /// 回写融合后的置信度
  static void set_confidence(auto_aim::Armor & a, double c) { a.confidence = c; }
};

}  // namespace tools

namespace auto_aim
{
/// 便捷别名（业务侧少打点字）
using TemporalIntegrator = tools::TemporalIntegrator<Armor>;
using TIConfig = tools::TIConfig;
}  // namespace auto_aim

#endif  // HZMIR_CORE_AUTO_AIM_DETECTOR_TEMPORAL_INTEGRATOR_HPP
