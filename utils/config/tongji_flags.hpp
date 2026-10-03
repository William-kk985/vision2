/**
 * @file utils/config/tongji_flags.hpp
 * @brief ⭐⭐⭐ **把 `--tongji` 总开关拆成独立槽位**（W100 · 原 B3/B4）
 *
 * ## 为什么拆
 * 原来 `--tongji=false` **一次性**改掉两处**互不相干**的东西：
 * ```
 * ① E3b  NIS 失败阈值：0.711（同济，χ²(4) 的 5% 下尾） vs 9.4877（χ²(4) 的 95% 分位）
 * ② E5   小陀螺判据：  ekf_x()[8]（同济，其实是半径 r → 分支永不可达） vs ekf_x()[7]（真正的角速度 w）
 * ```
 * ⚠️ 后果：想单独验证其中一个（比如"只改 NIS 阈值，不动小陀螺判据"）**做不到** ——
 *   只能两个一起开，**归因不清**（到底是哪个改动影响了命中率？）。
 *
 * ## 现在的三层优先级
 * ```
 * ① 单项显式指定（--nis-thresh=chi2 / --yaw-rate-src=x7）  ← 最高
 * ② 总开关（--tongji=false）                                ← 中
 * ③ 内置默认 = 同济行为                                     ← 最低
 * ```
 * ⭐ 语义：**单项给值就覆盖总开关**；不给就跟随总开关。
 *
 * ## 用法
 * ```cpp
 * const auto f = tools::resolve_tongji(
 *   cli.get<bool>("tongji"), cli.get<std::string>("nis-thresh"),
 *   cli.get<std::string>("yaw-rate-src"));
 * f.apply();                                    // 真正改全局状态
 * tools::logger()->info("[infantry] {}", f.describe());   // 一条日志说清最终生效的是哪套
 * ```
 */
#ifndef HZMIR_UTILS_CONFIG_TONGJI_FLAGS_HPP
#define HZMIR_UTILS_CONFIG_TONGJI_FLAGS_HPP

#include <string>

#include "core/auto_aim/planner/legacy.hpp"   // Aimer::set_tongji_compat
#include "utils/ekf/extended_kalman_filter.hpp"
#include "utils/log/logger.hpp"

namespace tools
{

/// NIS 失败阈值：同济原值 vs χ²(4) 95% 分位
enum class NisThresh { Tongji, Chi2Q95 };

/// 小陀螺判据用的 EKF 分量：`x[8]`（同济，实为半径）vs `x[7]`（真正的角速度 w）
enum class YawRateSrc { X8_Tongji, X7_Corrected };

/// @brief 解析后的「同济兼容」三项状态
struct TongjiFlags
{
  bool tongji = true;                        ///< 总开关（仅用于展示）
  NisThresh nis = NisThresh::Tongji;
  YawRateSrc yaw = YawRateSrc::X8_Tongji;

  /// @brief 是否有任何一项偏离了同济行为
  bool any_optimized() const
  {
    return nis != NisThresh::Tongji || yaw != YawRateSrc::X8_Tongji;
  }

  /// @brief 生效到全局状态
  void apply() const
  {
    if (nis == NisThresh::Chi2Q95)
      ExtendedKalmanFilter::use_chi2_q95_nis_threshold();
    else
      ExtendedKalmanFilter::use_tongji_nis_threshold();

    auto_aim::Aimer::set_tongji_compat(yaw == YawRateSrc::X8_Tongji);
  }

  /// @brief 一行说清最终生效的是哪套（⭐ 排查时最有用的一句话）
  std::string describe() const
  {
    const char * nis_s = (nis == NisThresh::Chi2Q95) ? "chi2=9.4877（优化）" : "tongji=0.711";
    const char * yaw_s = (yaw == YawRateSrc::X8_Tongji) ? "x[8]=r（同济）" : "x[7]=w（修正）";
    std::string s = "同济兼容 = ";
    s += tongji ? "开" : "关";
    s += "  |  NIS 阈值: ";
    s += nis_s;
    s += "  |  小陀螺判据: ";
    s += yaw_s;
    if (any_optimized()) {
      // ⭐ 区分"总开关关掉了"和"单项覆盖了"
      const bool by_master = (tongji == false);
      s += by_master ? "   ⚠️ 已偏离同济（--tongji=false）"
                     : "   ⚠️ 已偏离同济（单项覆盖）";
    }
    return s;
  }
};

/// @brief ⭐ 解析 `--tongji` / `--nis-thresh` / `--yaw-rate-src` 三层优先级
/// @param tongji    总开关（默认 true）
/// @param nis_opt   单项覆盖，`"tongji"` / `"chi2"` / 空 = 跟随总开关
/// @param yaw_opt   单项覆盖，`"x8"` / `"x7"` / 空 = 跟随总开关
inline TongjiFlags resolve_tongji(
  bool tongji, const std::string & nis_opt = "", const std::string & yaw_opt = "")
{
  TongjiFlags f;
  f.tongji = tongji;

  // ⭐ ② 总开关：false ⇒ 两项都切到"修正"值
  f.nis = tongji ? NisThresh::Tongji : NisThresh::Chi2Q95;
  f.yaw = tongji ? YawRateSrc::X8_Tongji : YawRateSrc::X7_Corrected;

  // ⭐ ① 单项覆盖（最高优先级）
  if (nis_opt == "chi2" || nis_opt == "chi2_q95") f.nis = NisThresh::Chi2Q95;
  else if (nis_opt == "tongji")                   f.nis = NisThresh::Tongji;
  else if (!nis_opt.empty())
    logger()->warn("[tongji] --nis-thresh 无法识别: {}（可选 tongji / chi2）", nis_opt);

  if (yaw_opt == "x7")          f.yaw = YawRateSrc::X7_Corrected;
  else if (yaw_opt == "x8")     f.yaw = YawRateSrc::X8_Tongji;
  else if (!yaw_opt.empty())
    logger()->warn("[tongji] --yaw-rate-src 无法识别: {}（可选 x8 / x7）", yaw_opt);

  return f;
}

}  // namespace tools

#endif  // HZMIR_UTILS_CONFIG_TONGJI_FLAGS_HPP
