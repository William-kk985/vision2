/**
 * @file utils/wheels/ballistic/ballistic_table.hpp
 * @brief ⭐ 轮子：弹道修正表（离线 RK4 建表 / 在线双线性插值）
 *
 * ## 为什么是「修正量」而不是「绝对解」
 * W12 实测：`rk4_drag` 单次要 **1467 µs**（占 10 ms 帧预算 14.7%）——
 * 而 `ideal`（斜抛无阻力）只要 **0.03 µs**。
 *
 * 但两者之差是**光滑、缓变**的 → 可以只表化**修正量**：
 * ```
 *   pitch_rk4(v0, d, h)  ≈  ideal(v0, d, h) + Δpitch(v0, d)
 *   t_fly_rk4(v0, d, h)  ≈  ideal_fly(v0, d) + Δt_fly(v0, d)
 * ```
 * **好处**：
 *   ① 表**极小**（修正量动态范围窄，可用 `float`）
 *   ② **任意高度 h 自动正确**（由 `ideal` 承担）
 *   ③ `unsolvable` 语义与 `ideal` 一致（由 `ideal` 判定）
 *
 * ⚠️ 代价：**假设 Δ 与 h 弱相关** —— 该假设由 `test_ballistic_table` 实测验证。
 *
 * 用法（离线建表，一次性）：
 * ```bash
 *   # 见 test/function/test_ballistic_table.cpp --build 分支
 * ```
 */
#ifndef HZMIR_UTILS_WHEELS_BALLISTIC_BALLISTIC_TABLE_HPP
#define HZMIR_UTILS_WHEELS_BALLISTIC_BALLISTIC_TABLE_HPP

#include <functional>
#include <string>
#include <vector>

#include "utils/wheels/ballistic/rk4_ballistic.hpp"

namespace tools
{

/// 一维轴定义
struct TableAxis
{
  double min = 0;
  double max = 0;
  double step = 0;
  int size() const { return static_cast<int>((max - min) / step) + 1; }
};

/// 修正量（`rk4 - ideal`）
struct BallisticCorrection
{
  double dpitch = 0;    ///< rad
  double dt_fly = 0;    ///< s
};

class BallisticTable
{
public:
  /// 基线模型签名（无阻力解析解）——**注入进来**，
  /// ⭐ 这样本文件不必 include `core/types.hpp`，保住「utils 零 core 依赖」纪律。
  using BaselineFn = std::function<BallisticResult(double v0, double d, double h)>;

  /// @brief 离线建表：对轴网格上的每个 (v0, d) 跑一次 RK4，与基线求差
  /// @param solver 已配好弹型的 RK4 求解器
  /// @param baseline 基线模型（通常传无阻力解析解的包装）
  /// @param v0_axis 初速轴
  /// @param dist_axis 水平距离轴
  /// @param height 建表用的目标高度（默认 0；修正量对 h 弱相关）
  /// @return 建表耗时（秒）
  double build(const BallisticSolver & solver, const BaselineFn & baseline,
               const TableAxis & v0_axis, const TableAxis & dist_axis, double height = 0.0);

  /// @brief 双线性插值查修正量（超出网格自动钳位到边界）
  BallisticCorrection correction(double v0, double distance) const;

  bool empty() const { return data_.empty(); }
  size_t entries() const { return data_.size(); }
  size_t bytes() const { return data_.size() * sizeof(BallisticCorrection); }

  const TableAxis & v0_axis() const { return v0_axis_; }
  const TableAxis & dist_axis() const { return dist_axis_; }

  /// @brief 存/取二进制（便于随代码提交一张预生成的表）
  bool save(const std::string & path) const;
  bool load(const std::string & path);

  /// @brief 网格外（v0/d 超出）的比例，用于判断表是否够大
  double out_of_range_ratio(double v0, double d) const;

private:
  TableAxis v0_axis_, dist_axis_;
  std::vector<BallisticCorrection> data_;   // 行优先：v0 外层，d 内层

  int i_v0(double v0) const;
  int i_d(double d) const;
  const BallisticCorrection & at(int iv, int id) const
  {
    return data_[iv * dist_axis_.size() + id];
  }
};

}  // namespace tools

#endif  // HZMIR_UTILS_WHEELS_BALLISTIC_BALLISTIC_TABLE_HPP
