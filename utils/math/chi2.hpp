/**
 * @file utils/math/chi2.hpp
 * @brief ⭐ 轮子：χ² 分布分位数（EKF 一致性检验用）
 *
 * ## 为什么需要它
 * EKF 的**一致性检验**（NIS / NEES）有严格理论：
 * ```
 *   若滤波器建模正确，则 NIS ~ χ²(m)，m = 观测维度
 *   于是「NIS 落在哪个分位」就是「滤波器是否可信」的判据
 * ```
 * 但代码里往往写成**魔法数字**（`9.21` / `4.0` / `0.71`），看不出含义、也容易写错。
 *
 * ⭐ 实测过的例子（W9）：
 * `extended_kalman_filter.cpp` 用 `0.711` 当"95% 置信"阈值 ——
 * 而 `0.711` 其实是 χ²(4) 的 **5% 下尾**，95% 分位是 **9.488**（差 13 倍）。
 *
 * ## 用法
 * ```cpp
 *   if (nis > tools::chi2_q95(4)) { /* 超出 95% 分位 → 可疑 *\/ }
 *   // 或
 *   const double mean = tools::chi2_mean(4);   // 4.0，作"优于平均"判据
 * ```
 *
 * ## 数据来源
 * 标准 χ² 分位表（Abramowitz & Stegun / 常用统计表），
 * dof 1~10 直接查表（精确到 4 位有效数字）；更大自由度用 Wilson–Hilferty 近似。
 */
#ifndef HZMIR_UTILS_MATH_CHI2_HPP
#define HZMIR_UTILS_MATH_CHI2_HPP

#include <array>
#include <cmath>

namespace tools
{

/// χ² 的常用分位（自由度 = `dof`）
struct Chi2Quantiles
{
  double mean = 0;   ///< 均值 = dof
  double q05 = 0;    ///< 5% 下尾（"好得可疑"的下界）
  double q50 = 0;    ///< 中位数
  double q95 = 0;    ///< 95% 分位（**一致性检验的通过上界**）
  double q99 = 0;    ///< 99% 分位（严重不一致）
};

namespace detail
{
// dof 1~10 的分位表 {q05, q50, q95, q99}
inline constexpr std::array<std::array<double, 4>, 10> CHI2_TABLE = {{
  //  dof=1
  {{0.003932, 0.4549, 3.8415, 6.6349}},
  //  dof=2
  {{0.10259, 1.3863, 5.9915, 9.2103}},
  //  dof=3
  {{0.35185, 2.3660, 7.8147, 11.3449}},
  //  dof=4   ⭐ EKF 观测维度 m=4（yaw, pitch, distance, angle）
  {{0.71072, 3.3567, 9.4877, 13.2767}},
  //  dof=5
  {{1.14548, 4.3515, 11.0705, 15.0863}},
  //  dof=6
  {{1.63538, 5.3481, 12.5916, 16.8119}},
  //  dof=7
  {{2.16735, 6.3458, 14.0671, 18.4753}},
  //  dof=8
  {{2.73264, 7.3441, 15.5073, 20.0902}},
  //  dof=9
  {{3.32511, 8.3428, 16.9190, 21.6660}},
  //  dof=10
  {{3.94030, 9.3418, 18.3070, 23.2093}},
}};

/// Wilson–Hilferty 近似（dof 较大时足够准）
inline Chi2Quantiles chi2_wilson_hilferty(int dof)
{
  const double k = static_cast<double>(dof);
  // z_p 为正态分位
  auto approx = [k](double z) {
    const double t = 1.0 - 2.0 / (9.0 * k) + z * std::sqrt(2.0 / (9.0 * k));
    return k * t * t * t;
  };
  return {k, approx(-1.6449), approx(0.0), approx(1.6449), approx(2.3263)};
}
}  // namespace detail

/// @brief χ²(dof) 的常用分位
/// @param dof 自由度（= 观测维度 m）；<=0 视为 1
inline Chi2Quantiles chi2_quantiles(int dof)
{
  if (dof <= 0) dof = 1;
  if (dof <= 10) {
    const auto & r = detail::CHI2_TABLE[static_cast<size_t>(dof - 1)];
    return {static_cast<double>(dof), r[0], r[1], r[2], r[3]};
  }
  return detail::chi2_wilson_hilferty(dof);
}

/// @brief 95% 分位 —— **一致性检验的通过上界**（推荐用它当 `nis_threshold`）
inline double chi2_q95(int dof) { return chi2_quantiles(dof).q95; }

/// @brief 5% 下尾 —— "好得可疑"的下界（⚠️ 别把它当 95% 分位用）
inline double chi2_q05(int dof) { return chi2_quantiles(dof).q05; }

/// @brief 均值 = dof —— 作「优于平均」的质量判据
inline double chi2_mean(int dof) { return static_cast<double>(dof <= 0 ? 1 : dof); }

/// @brief 中位数
inline double chi2_median(int dof) { return chi2_quantiles(dof).q50; }

}  // namespace tools

#endif  // HZMIR_UTILS_MATH_CHI2_HPP
