#ifndef TOOLS__EXTENDED_KALMAN_FILTER_HPP
#define TOOLS__EXTENDED_KALMAN_FILTER_HPP

#include <Eigen/Dense>
#include <deque>
#include <functional>
#include <map>

namespace tools
{
class ExtendedKalmanFilter
{
public:
  Eigen::VectorXd x;
  Eigen::MatrixXd P;

  ExtendedKalmanFilter() = default;

  ExtendedKalmanFilter(
    const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a + b; });

  Eigen::VectorXd predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q);

  Eigen::VectorXd predict(
    const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f);

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  Eigen::VectorXd update(
    const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
    std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract =
      [](const Eigen::VectorXd & a, const Eigen::VectorXd & b) { return a - b; });

  std::map<std::string, double> data;  //卡方检验数据

  // ⭐ W24：一致性检验阈值（观测维度 m = 4）
  //   默认 = χ²(4) 的 95% 分位 = 9.4877（**理论上正确的**一致性上界）
  //   历史行为 = χ²(4) 的 5% 下尾 = 0.7107（原代码误用，会让 ~49% 的帧判"失败"）
  /// @brief 当前一致性检验阈值
  /// ⭐⭐ **默认 = 同济原值 `0.711`**（同济源码为准）
  ///   我们的分析认为 `0.711` 是 χ²(4) 的 **5% 下尾**、应为 95% 分位 `9.4877`，
  ///   但那是**优化建议**，需要自行开启：`use_chi2_q95_nis_threshold()`
  /// ⚠️⚠️ **B14：必须是"非 inline 的静态成员"（定义在 .cpp）**
  ///   原来写成 `inline static` → 在**静态库**架构下每个 `.a` 会各持一份副本：
  ///   `Target` 在 `hzmir_auto_aim` 里，它读到的是**自己那份**，
  ///   测试改了 `hzmir_utils` 那份也不生效（实测两次失败率完全相同 52.9%）。
  static double nis_fail_threshold();
  static void set_nis_fail_threshold(double v);
  /// ⭐ 默认值 = 同济原值（0.711 = χ²(4) 的 5% 下尾）
  static void use_tongji_nis_threshold();
  /// ⭐ **优化建议**：改用 χ²(4) 的 95% 分位（9.4877）—— 理论上正确的一致性上界
  static void use_chi2_q95_nis_threshold();
  std::deque<int> recent_nis_failures{0};
  size_t window_size = 100;
  double last_nis;

private:
  Eigen::MatrixXd I;
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add;

  int nees_count_ = 0;
  int nis_count_ = 0;
  int total_count_ = 0;
};

}  // namespace tools

#endif  // TOOLS__EXTENDED_KALMAN_FILTER_HPP