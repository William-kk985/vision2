#include "extended_kalman_filter.hpp"

#include "utils/math/chi2.hpp"

namespace tools
{
// ⭐ B14：唯一定义（不放头文件 —— `inline static` 会在静态库边界上被复制）
// ⭐⭐ 默认 = **同济原值**（0.711）；我们的修正（χ²(4) 的 95% 分位 9.4877）是**可选项**
static double g_nis_fail_threshold = 0.711;

double ExtendedKalmanFilter::nis_fail_threshold() { return g_nis_fail_threshold; }
void ExtendedKalmanFilter::set_nis_fail_threshold(double v) { g_nis_fail_threshold = v; }
void ExtendedKalmanFilter::use_tongji_nis_threshold() { g_nis_fail_threshold = 0.711; }
void ExtendedKalmanFilter::use_chi2_q95_nis_threshold() { g_nis_fail_threshold = chi2_q95(4); }
}  // namespace tools

#include <numeric>

namespace tools
{
ExtendedKalmanFilter::ExtendedKalmanFilter(
  const Eigen::VectorXd & x0, const Eigen::MatrixXd & P0,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> x_add)
: x(x0), P(P0), I(Eigen::MatrixXd::Identity(x0.rows(), x0.rows())), x_add(x_add)
{
  data["residual_yaw"] = 0.0;
  data["residual_pitch"] = 0.0;
  data["residual_distance"] = 0.0;
  data["residual_angle"] = 0.0;
  data["nis"] = 0.0;
  data["nees"] = 0.0;
  data["nis_fail"] = 0.0;
  data["nees_fail"] = 0.0;
  data["recent_nis_failures"] = 0.0;
}

Eigen::VectorXd ExtendedKalmanFilter::predict(const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q)
{
  return predict(F, Q, [&](const Eigen::VectorXd & x) { return F * x; });
}

Eigen::VectorXd ExtendedKalmanFilter::predict(
  const Eigen::MatrixXd & F, const Eigen::MatrixXd & Q,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> f)
{
  P = F * P * F.transpose() + Q;
  x = f(x);
  return x;
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  return update(z, H, R, [&](const Eigen::VectorXd & x) { return H * x; }, z_subtract);
}

Eigen::VectorXd ExtendedKalmanFilter::update(
  const Eigen::VectorXd & z, const Eigen::MatrixXd & H, const Eigen::MatrixXd & R,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &)> h,
  std::function<Eigen::VectorXd(const Eigen::VectorXd &, const Eigen::VectorXd &)> z_subtract)
{
  Eigen::VectorXd x_prior = x;
  Eigen::MatrixXd P_prior = P;   // ⭐ W9：存先验协方差（原代码只存了 x_prior，没存 P_prior）
  Eigen::MatrixXd K = P * H.transpose() * (H * P * H.transpose() + R).inverse();

  // Stable Compution of the Posterior Covariance
  // https://github.com/rlabbe/Kalman-and-Bayesian-Filters-in-Python/blob/master/07-Kalman-Filter-Math.ipynb
  P = (I - K * H) * P * (I - K * H).transpose() + K * R * K.transpose();

  x = x_add(x, K * z_subtract(z, h(x)));

  /// 卡方检验
  Eigen::VectorXd residual = z_subtract(z, h(x));
  // 新增检验
  Eigen::MatrixXd S = H * P * H.transpose() + R;
  double nis = residual.transpose() * S.inverse() * residual;
  double nees = (x - x_prior).transpose() * P.inverse() * (x - x_prior);

  // ⭐ W9：**并行计算「正确的先验 NIS/NEES」**用于对比（附加字段，不改变原逻辑）
  //   理论上 NIS ~ χ²(m)，m = 观测维度；χ²(4): 均值=4, 95%分位=9.488, 5%下尾=0.711
  Eigen::VectorXd residual_prior = z_subtract(z, h(x_prior));
  Eigen::MatrixXd S_prior = H * P_prior * H.transpose() + R;
  double nis_prior = residual_prior.transpose() * S_prior.inverse() * residual_prior;
  double nees_prior =
    (x - x_prior).transpose() * P_prior.inverse() * (x - x_prior);
  data["nis_prior"] = nis_prior;
  data["nees_prior"] = nees_prior;

  // ⭐ W24：阈值改用 χ²(4) 分位表达（不再是魔法数字）
  //   · 观测维度 m = 4（yaw, pitch, distance, angle）
  //   · ⚠️ 原值 0.711 其实是 χ²(4) 的 **5% 下尾**（"好得可疑"的下界），
  //     被误当成 95% 分位用 → W9 实测 **49% 的帧被判「失败」**。
  //     正确的一致性上界是 95% 分位 = 9.4877（差 13.3 倍）。
  //   · `nis_fail_threshold()` 可在运行期改（默认已换成正确值）
  const double nis_threshold = nis_fail_threshold();
  const double nees_threshold = nis_fail_threshold();

  if (nis > nis_threshold) nis_count_++, data["nis_fail"] = 1;
  if (nees > nees_threshold) nees_count_++, data["nees_fail"] = 1;
  total_count_++;
  last_nis = nis;

  recent_nis_failures.push_back(nis > nis_threshold ? 1 : 0);

  if (recent_nis_failures.size() > window_size) {
    recent_nis_failures.pop_front();
  }

  int recent_failures = std::accumulate(recent_nis_failures.begin(), recent_nis_failures.end(), 0);
  double recent_rate = static_cast<double>(recent_failures) / recent_nis_failures.size();

  data["residual_yaw"] = residual[0];
  data["residual_pitch"] = residual[1];
  data["residual_distance"] = residual[2];
  data["residual_angle"] = residual[3];
  data["nis"] = nis;
  data["nees"] = nees;
  data["recent_nis_failures"] = recent_rate;

  return x;
}

}  // namespace tools