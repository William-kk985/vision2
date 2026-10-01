// test/function/test_ekf_nis.cpp —— ⭐ W9：EKF 的 NIS/NEES 一致性检验
//
// **完全零硬件**：EKF 的观测是 `Armor`（几何量），不是图像。
// 做法（EKF 标准测试法 filter consistency）：
//   ① 造一个已知运动的目标（匀速旋转）
//   ② 用解析前向模型 + 高斯噪声合成观测
//   ③ 喂给 Target::update()，收集 NIS / NIS_prior
//   ④ 与理论值对比：NIS ~ χ²(m)，χ²(4): 均值=4, 95%分位=9.488, 5%下尾=0.711
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <vector>

#include "core/types.hpp"
#include "utils/math/chi2.hpp"
#include "core/auto_aim/target/target.hpp"
#include "utils/math/math_tools.hpp"

using namespace auto_aim;

// Armor 没有默认构造 → 用 Lightbar 造一块（字段随后覆盖）
static Armor make_bare_armor()
{
  Lightbar l, r;
  l.angle = r.angle = 0.1; l.length = r.length = 1.0;
  l.width = r.width = 0.2; l.ratio = r.ratio = 5.0;
  l.center = {0.f, 0.f};  l.top = {0.f, 0.f};  l.bottom = {0.f, 1.f};
  r.center = {0.5f, 0.f}; r.top = {0.5f, 0.f}; r.bottom = {0.5f, 1.f};
  return Armor(l, r);
}

// χ²(4) 的理论值
constexpr double CHI2_4_MEAN = 4.0;
constexpr double CHI2_4_Q95 = 9.488;
constexpr double CHI2_4_TAIL05 = 0.711;

static double quantile(std::vector<double> v, double q)
{
  if (v.empty()) return 0;
  std::sort(v.begin(), v.end());
  return v[std::min(v.size() - 1, size_t(q * v.size()))];
}

int main()
{
  std::mt19937 rng(42);
  std::normal_distribution<double> noise(0.0, 1.0);

  // ⭐ 已知运动：水平距离 4 m，角速度 1 rad/s，半径 0.2 m，高度差 0.1 m
  Target target(4.0, 1.0, 0.2, 0.1);

  const double dt = 0.01;
  const double yaw_sigma = 0.002;    // 2 mrad 观测噪声（约 0.11°）
  const double dist_sigma = 0.01;    // 1 cm

  std::vector<double> nis_post, nis_prior;
  const int N = 3000;
  const int WARMUP = 500;        // ⭐ 丢弃收敛期（P0=0，前期 NIS 无意义）
  int jumped_count = 0, outlier_count = 0;

  for (int k = 0; k < N; ++k) {
    target.predict(dt);

    // 解析前向模型（目标 0 号装甲板）作为"真值"
    auto xyza = target.armor_xyza_list()[0];
    Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));

    // 合成带噪声的观测（Armor 的 ypr_in_world / ypd_in_world 是 EKF 实际消费的字段）
    Armor obs = make_bare_armor();
    obs.name = ArmorName::three;
    obs.color = Color::red;
    obs.ypr_in_world = Eigen::Vector3d(xyza[3] + yaw_sigma * noise(rng), 0, 0);
    obs.ypd_in_world = Eigen::Vector3d(
      ypd[0] + yaw_sigma * noise(rng),
      ypd[1] + yaw_sigma * noise(rng),
      ypd[2] + dist_sigma * noise(rng));

    target.update(obs);
    if (target.jumped) jumped_count++;

    if (k < WARMUP) continue;    // ⭐ 跳过热身期

    const auto & d = target.ekf().data;
    auto it_post = d.find("nis");
    auto it_pri = d.find("nis_prior");
    if (it_post != d.end() && std::isfinite(it_post->second)) {
      nis_post.push_back(it_post->second);
      if (it_post->second > 100.0) outlier_count++;   // ⭐ 离群（多为 id 切换）
    }
    if (it_pri != d.end() && std::isfinite(it_pri->second)) nis_prior.push_back(it_pri->second);
  }

  auto stat = [](const std::vector<double> & v) {
    double s = 0;
    for (auto x : v) s += x;
    return s / std::max<size_t>(1, v.size());
  };

  std::printf("样本数: %zu\n\n", nis_post.size());
  std::printf("%-34s %10s %10s\n", "", "NIS(现状)", "NIS(先验)");
  std::printf("%-34s %10.4f %10.4f\n", "均值", stat(nis_post), stat(nis_prior));
  std::printf("%-34s %10.4f %10.4f\n", "5%  ~  95% 区间下沿", quantile(nis_post, 0.05),
              quantile(nis_prior, 0.05));
  std::printf("%-34s %10.4f %10.4f\n", "中位数", quantile(nis_post, 0.50),
              quantile(nis_prior, 0.50));
  std::printf("%-34s %10.4f %10.4f\n", "75% 分位", quantile(nis_post, 0.75),
              quantile(nis_prior, 0.75));
  std::printf("%-34s %10.4f %10.4f\n", "95% 分位", quantile(nis_post, 0.95),
              quantile(nis_prior, 0.95));
  std::printf("%-34s %10.4f %10.4f\n", "99% 分位", quantile(nis_post, 0.99),
              quantile(nis_prior, 0.99));
  std::printf("\n%-34s %10d  (id 切换：装甲板匹配换板)\n", "jumped 次数", jumped_count);
  std::printf("%-34s %10d  (%.1f%%，NIS>100 的离群帧)\n", "离群帧数", outlier_count,
              100.0 * outlier_count / std::max<size_t>(1, nis_post.size()));
  std::printf("\n%-34s %10.4f\n", "χ²(4) 理论均值", CHI2_4_MEAN);
  std::printf("%-34s %10.4f\n", "χ²(4) 理论 95% 分位", CHI2_4_Q95);
  std::printf("%-34s %10.4f\n", "χ²(4) 理论 5% 下尾（⭐ W24 前的旧阈值）", CHI2_4_TAIL05);

  const double m_post = stat(nis_post), m_prior = stat(nis_prior);
  // ═══════════════════════════════════════════════════════════
  // ⭐ W24：阈值改动的**影响量化**
  //   旧 0.7107（χ²(4) 的 5% 下尾，被误当 95% 分位）→ 约 49% 帧判"失败"
  //   新 9.4877（χ²(4) 的 95% 分位，理论上正确）    → 期望约 5% 帧判"失败"
  // ═══════════════════════════════════════════════════════════
  // ⭐ W24：阈值用 χ² 分位表达后的**诚实结论**
  // ═══════════════════════════════════════════════════════════
  std::printf("\n--- W24: 阈值统一（chi2 分位）---\n");
  {
    const double tail = tools::chi2_q05(4);     // 0.71072
    const double q95 = tools::chi2_q95(4);      // 9.4877
    std::printf("     旧值 0.711  = chi2(4) 5%% 下尾  = %.5f\n", tail);
    std::printf("     新值 %.4f = chi2(4) 95%% 分位\n", q95);
    std::printf("     => 分位正确性是**纯数学**结论：旧值确实用错了分位\n");

    // 换阈值能救多少帧？—— 用当前合成数据实测
    auto count_fail = [&](double th) {
      tools::ExtendedKalmanFilter::set_nis_fail_threshold(th);
      Target t2(4.0, 1.0, 0.2, 0.1);
      std::mt19937 rng2(42);
      std::normal_distribution<double> nz(0.0, 1.0);
      int fail = 0, total = 0;
      for (int k = 0; k < 2500; ++k) {
        t2.predict(0.01);
        auto xyza = t2.armor_xyza_list()[0];
        Eigen::Vector3d ypd = tools::xyz2ypd(xyza.head(3));
        Armor o = make_bare_armor();
        o.name = ArmorName::three;
        o.color = Color::red;
        o.ypr_in_world = Eigen::Vector3d(xyza[3] + 0.002 * nz(rng2), 0, 0);
        o.ypd_in_world = Eigen::Vector3d(ypd[0] + 0.002 * nz(rng2), ypd[1] + 0.002 * nz(rng2),
                                         ypd[2] + 0.01 * nz(rng2));
        t2.update(o);
        if (k < 500) continue;
        ++total;
        const auto & dd = t2.ekf().data;
        auto it = dd.find("nis");
        if (it != dd.end() && it->second > th) ++fail;
      }
      return double(fail) / total;
    };
    const double r_old = count_fail(tail);
    const double r_new = count_fail(q95);
    std::printf("     实测失败率：旧阈值 %.1f%%  新阈值 %.1f%%\n", r_old * 100, r_new * 100);
    if (std::abs(r_old - r_new) < 0.02) {
      std::printf("     [!!] ⚠️ 两者相同 -> **NIS 分布是双峰的**（见上：中位数 0.0075 / 75%% 分位 15.9）\n");
      std::printf("          => 0.71~9.49 之间**几乎没有样本**，换阈值不改变计数\n");
      std::printf("          => 「分位对不对」= 纯数学（成立）；「能救多少帧」**必须用真实数据测**\n");
    } else {
      std::printf("     [OK] 失败率显著下降（%.1f%% -> %.1f%%）\n", r_old * 100, r_new * 100);
    }
    tools::ExtendedKalmanFilter::set_nis_fail_threshold(q95);   // 还原为正确值
  }

  std::printf("\n" "─ 判读 " "──────────────────────────────────────────────────\n");
  std::printf("① 均值被离群值拉高 —— 中位数才是「正常帧」的 NIS\n");
  std::printf("   中位数 %.4f vs χ²(4) 中位数 3.357 → %s\n", quantile(nis_post, 0.50),
              quantile(nis_post, 0.50) < 1.0 ? "❌ 小几百倍：R 设得太大，滤波器几乎不信观测"
                                             : "✅ 量级相当");
  std::printf("   （R_dig[2] = log(|d|+1)+1 ≈ 1 → 距离 sigma ≈ 1 米！）\n\n");
  std::printf("② 均值应 ≈ χ²(4)=4.00\n");
  std::printf("   现状(后验) = %.3f  → %s\n", m_post,
              (m_post < 0.5 * CHI2_4_MEAN ? "❌ 严重低估" : "🔶 偏低"));
  std::printf("   先验       = %.3f  → %s\n", m_prior,
              (std::abs(m_prior - CHI2_4_MEAN) < 1.0 ? "✅ 与理论相符" : "🔶 需核对"));
  std::printf("② 现状阈值 0.711 = χ²(4) 的 5%% 下尾 → NIS 有 %.1f%% 的帧被误判为\"失败\"\n",
              100.0 * [&] {
                size_t c = 0;
                for (auto x : nis_post) if (x > CHI2_4_TAIL05) c++;
                return double(c) / nis_post.size();
              }());
  std::printf("③ 正确阈值应为 95%% 分位 = %.3f（差 %.1f 倍）\n", CHI2_4_Q95,
              CHI2_4_Q95 / CHI2_4_TAIL05);
  return 0;
}
