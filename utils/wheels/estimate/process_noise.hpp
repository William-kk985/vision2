/**
 * @file utils/wheels/estimate/process_noise.hpp
 * @brief ⭐ 轮子：**EKF 过程噪声 Q 的调整策略**（零业务依赖）
 *
 * ## 来源与演化
 * 原为 `Target` 的策略槽位（`IProcessNoiseAdapter`，W23 迁入）。
 * ⭐ **W43 改为独立轮子**（去掉虚接口）—— `Target` 恢复同济原样（硬编码 `v1/v2`）。
 *
 * ## 背景
 * 同济的做法（`Target::predict`）：
 * ```cpp
 * double v1, v2;
 * if (name == ArmorName::outpost) { v1 = 10;  v2 = 0.1; }
 * else                            { v1 = 100; v2 = 400; }
 * ```
 * **硬编码、不自适应**。本轮的"自适应"就是**按 NIS 反馈调整这两个数**。
 *
 * ⚠️⚠️ **原实现有个方向性 bug（W23 已修）**：
 * ```
 * 普通目标默认 v2=400，但原上限写 300/400
 * → NIS 变大（"应该提高 Q"）时：v2 = min(300, 400*1.1) = 300   ← 反而**降低**了！
 * 修法：上下限可配且默认高于基线（v2_max = 800）
 * ```
 */
#ifndef HZMIR_UTILS_WHEELS_ESTIMATE_PROCESS_NOISE_HPP
#define HZMIR_UTILS_WHEELS_ESTIMATE_PROCESS_NOISE_HPP

#include <algorithm>

namespace utils::wheels
{

/// ⭐ 固定噪声（= 同济行为）：什么都不改
struct FixedNoise
{
  void adapt(double /*nis*/, double & /*v1*/, double & /*v2*/) const {}
};

/// ⭐ NIS 反馈自适应：NIS 大 → 提高 Q（信任观测更多）
class NisBasedAdapter
{
public:
  /// ⚠️ 上下限**必须高于基线默认**，否则会出现"想提高却降低了"的反向 bug（见文件头）
  NisBasedAdapter(
    double v1_min = 20.0, double v1_max = 150.0, double v2_min = 80.0, double v2_max = 800.0)
  : v1_min_(v1_min), v1_max_(v1_max), v2_min_(v2_min), v2_max_(v2_max)
  {
  }

  /// @param nis 归一化新息平方（来自 EKF 的 `data["nis"]`）
  /// @param v1  加速度方差（就地修改）
  /// @param v2  角加速度方差（就地修改）
  void adapt(double nis, double & v1, double & v2) const
  {
    if (nis <= 0.0) return;                       // 无有效 NIS → 不动
    if (nis > kStrong * kNisQ95) {                // 明显不一致 → 大幅提高
      v1 = std::min(v1_max_, v1 * 1.2);
      v2 = std::min(v2_max_, v2 * 1.2);
    } else if (nis > kNisQ95) {                   // 不一致 → 提高
      v1 = std::min(v1_max_, v1 * 1.1);
      v2 = std::min(v2_max_, v2 * 1.1);
    } else if (nis < kNisQ05) {                   // 过于一致（滤波器太自信）→ 降低
      v1 = std::max(v1_min_, v1 * 0.8);
      v2 = std::max(v2_min_, v2 * 0.8);
    }
    v1 = std::max(v1_min_, std::min(v1_max_, v1));
    v2 = std::max(v2_min_, std::min(v2_max_, v2));
  }

  double v1_min() const { return v1_min_; }
  double v1_max() const { return v1_max_; }
  double v2_min() const { return v2_min_; }
  double v2_max() const { return v2_max_; }

  // ⭐ χ²(4) 的分位（与 `utils/math/chi2.hpp` 一致，这里内联避免耦合）
  static constexpr double kNisQ05 = 0.711;    ///< χ²(4) 5% 下尾
  static constexpr double kNisQ95 = 9.488;    ///< χ²(4) 95% 分位
  static constexpr double kStrong = 1.5;      ///< "明显不一致"的倍数

private:
  double v1_min_, v1_max_, v2_min_, v2_max_;
};

}  // namespace utils::wheels

#endif  // HZMIR_UTILS_WHEELS_ESTIMATE_PROCESS_NOISE_HPP
